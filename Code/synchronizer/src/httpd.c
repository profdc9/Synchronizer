/* httpd.c */

/*
   Copyright (c) 2026 Daniel Marks

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "httpd.h"

typedef struct _http_conn
{
  struct tcp_pcb *pcb;
  bool     used;
  uint16_t reqlen;
  char     req[HTTP_REQ_MAX];
  char     hdr[192];
  uint16_t hdrlen, hdrsent;
  char     body[HTTP_BODY_MAX];   /* scratch for dynamic responses      */
  const char *out;                /* what to send - body, or flash      */
  uint32_t outlen, outsent;
  bool     done;
} http_conn;

static http_conn conns[HTTP_MAX_CONNS];
static struct tcp_pcb *listener;
static uint32_t req_count;

/* --- query strings ----------------------------------------------------- */

static int hexval(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool http_query_get(const char *q, const char *key, char *out, uint32_t outsz)
{
  uint32_t klen = (uint32_t)strlen(key);
  const char *p = q;

  if (!q || !*q || outsz == 0u) return false;
  while (*p)
  {
    const char *amp = strchr(p, '&');
    const char *end = amp ? amp : (p + strlen(p));
    if ((uint32_t)(end - p) > klen && p[klen] == '=' && strncmp(p, key, klen) == 0)
    {
      const char *v = p + klen + 1;
      uint32_t n = 0;
      while (v < end && n + 1u < outsz)
      {
        if (*v == '%' && (v + 2) < end && hexval(v[1]) >= 0 && hexval(v[2]) >= 0)
        { out[n++] = (char)((hexval(v[1]) << 4) | hexval(v[2])); v += 3; }
        else if (*v == '+') { out[n++] = ' '; v++; }
        else out[n++] = *v++;
      }
      out[n] = '\0';
      return true;
    }
    if (!amp) break;
    p = amp + 1;
  }
  return false;
}

long http_query_int(const char *q, const char *key, long fallback)
{
  char buf[24], *end;
  long v;
  if (!http_query_get(q, key, buf, sizeof(buf))) return fallback;
  v = strtol(buf, &end, 10);
  /* Text that is not a number at all falls back rather than reading as
     zero, which would otherwise turn "?us=oops" into a silent no-op. */
  if (end == buf) return fallback;
  return v;
}

/* --- connection plumbing ----------------------------------------------- */

static const char *status_text(int s)
{
  switch (s)
  {
    case 200: return "OK";
    case 202: return "Accepted";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 503: return "Service Unavailable";
  }
  return "Error";
}

static void conn_release(http_conn *c)
{
  c->used = false;
  c->pcb  = NULL;
}

static void conn_close(http_conn *c)
{
  struct tcp_pcb *pcb = c->pcb;
  if (!pcb) { conn_release(c); return; }
  tcp_arg(pcb, NULL);
  tcp_recv(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_err(pcb, NULL);
  tcp_poll(pcb, NULL, 0);
  conn_release(c);
  if (tcp_close(pcb) != ERR_OK) tcp_abort(pcb);
}

/* Push as much of the header and body as the send buffer will take. */
static void conn_push(http_conn *c)
{
  err_t e;

  if (!c->pcb) return;

  while (c->hdrsent < c->hdrlen)
  {
    uint16_t room = tcp_sndbuf(c->pcb);
    uint16_t n    = (uint16_t)(c->hdrlen - c->hdrsent);
    if (room == 0u) { tcp_output(c->pcb); return; }
    if (n > room) n = room;
    e = tcp_write(c->pcb, c->hdr + c->hdrsent, n, TCP_WRITE_FLAG_COPY);
    if (e != ERR_OK) { tcp_output(c->pcb); return; }
    c->hdrsent = (uint16_t)(c->hdrsent + n);
  }

  while (c->outsent < c->outlen)
  {
    uint16_t room = tcp_sndbuf(c->pcb);
    uint32_t left = c->outlen - c->outsent;
    uint16_t n    = (left > 0xFFFFu) ? 0xFFFFu : (uint16_t)left;
    if (room == 0u) { tcp_output(c->pcb); return; }
    if (n > room) n = room;
    /* The body is either this connection's own buffer or a string in
       flash; both outlive the send, so lwIP need not copy. */
    e = tcp_write(c->pcb, c->out + c->outsent, n, 0);
    if (e != ERR_OK) { tcp_output(c->pcb); return; }
    c->outsent += n;
  }

  tcp_output(c->pcb);
  if (c->outsent >= c->outlen && c->hdrsent >= c->hdrlen) c->done = true;
}

static void serve(http_conn *c)
{
  char *sp1, *sp2, *q;
  const char *method, *path, *query = "";
  http_response r;

  req_count++;

  /* "METHOD /path?query HTTP/1.1" */
  sp1 = strchr(c->req, ' ');
  if (!sp1) { conn_close(c); return; }
  *sp1 = '\0';
  method = c->req;
  path   = sp1 + 1;
  sp2 = strchr(path, ' ');
  if (sp2) *sp2 = '\0';
  q = strchr((char *)path, '?');
  if (q) { *q = '\0'; query = q + 1; }

  r.status = 404; r.ctype = "text/plain"; r.body = "not found"; r.len = 9;
  http_dispatch(method, path, query, c->body, (uint32_t)sizeof(c->body), &r);

  c->hdrlen = (uint16_t)snprintf(c->hdr, sizeof(c->hdr),
      "HTTP/1.1 %d %s\r\n"
      "Content-Type: %s\r\n"
      "Content-Length: %lu\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n"
      "\r\n",
      r.status, status_text(r.status), r.ctype, (unsigned long)r.len);
  c->hdrsent = 0;
  c->out     = r.body;
  c->outlen  = r.len;
  c->outsent = 0;
  c->done    = false;
  conn_push(c);
}

static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  http_conn *c = (http_conn *)arg;
  (void)pcb; (void)len;
  if (!c) return ERR_OK;
  conn_push(c);
  if (c->done) conn_close(c);
  return ERR_OK;
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  http_conn *c = (http_conn *)arg;

  if (!c) { if (p) pbuf_free(p); return ERR_OK; }
  if (!p) { conn_close(c); return ERR_OK; }          /* peer closed */
  if (err != ERR_OK) { pbuf_free(p); conn_close(c); return ERR_OK; }

  {
    uint16_t room = (uint16_t)(HTTP_REQ_MAX - 1u - c->reqlen);
    uint16_t n    = (p->tot_len > room) ? room : p->tot_len;
    pbuf_copy_partial(p, c->req + c->reqlen, n, 0);
    c->reqlen = (uint16_t)(c->reqlen + n);
    c->req[c->reqlen] = '\0';
  }
  tcp_recved(pcb, p->tot_len);
  pbuf_free(p);

  /* Wait for the end of the headers.  Any body is ignored - every
     mutation on this device carries its parameters in the query string. */
  if (strstr(c->req, "\r\n\r\n") || strstr(c->req, "\n\n"))
    serve(c);
  else if (c->reqlen >= HTTP_REQ_MAX - 1u)
    conn_close(c);

  return ERR_OK;
}

static void on_err(void *arg, err_t err)
{
  http_conn *c = (http_conn *)arg;
  (void)err;
  if (c) conn_release(c);      /* the pcb is already gone */
}

static err_t on_poll(void *arg, struct tcp_pcb *pcb)
{
  http_conn *c = (http_conn *)arg;
  (void)pcb;
  if (!c) return ERR_OK;
  if (c->done) { conn_close(c); return ERR_OK; }
  conn_push(c);                /* nudge a stalled send */
  return ERR_OK;
}

static err_t on_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  int i;
  (void)arg;

  if (err != ERR_OK || !pcb) return ERR_VAL;

  for (i = 0; i < HTTP_MAX_CONNS; i++) if (!conns[i].used) break;
  if (i == HTTP_MAX_CONNS) { tcp_abort(pcb); return ERR_ABRT; }

  memset(&conns[i], '\0', sizeof(conns[i]));
  conns[i].used = true;
  conns[i].pcb  = pcb;

  tcp_arg(pcb, &conns[i]);
  tcp_recv(pcb, on_recv);
  tcp_sent(pcb, on_sent);
  tcp_err(pcb, on_err);
  tcp_poll(pcb, on_poll, 8);       /* every 4 s */
  tcp_nagle_disable(pcb);
  return ERR_OK;
}

void httpd_init(uint16_t port)
{
  struct tcp_pcb *pcb;

  if (listener) return;

  cyw43_arch_lwip_begin();
  pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
  if (pcb)
  {
    if (tcp_bind(pcb, IP_ANY_TYPE, port) == ERR_OK)
    {
      listener = tcp_listen_with_backlog(pcb, 2);
      if (listener) tcp_accept(listener, on_accept);
      else tcp_close(pcb);
    }
    else tcp_close(pcb);
  }
  cyw43_arch_lwip_end();
}

bool httpd_running(void)     { return listener != NULL; }
uint32_t httpd_requests(void) { return req_count; }
