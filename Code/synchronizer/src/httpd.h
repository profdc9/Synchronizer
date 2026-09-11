/* httpd.h - a very small HTTP/1.1 server on raw lwIP TCP */

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

#ifndef _HTTPD_H
#define _HTTPD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HTTP_MAX_CONNS   3
#define HTTP_REQ_MAX     1024
#define HTTP_BODY_MAX    4096

/* What a handler fills in.  `body` may point at flash - the page itself
   does - or at the scratch buffer the dispatcher was handed. */
typedef struct _http_response
{
  int         status;        /* 200, 202, 400, 404, 500                  */
  const char *ctype;
  const char *body;
  uint32_t    len;
} http_response;

/* Implemented by webui.c.  Runs in lwIP callback context, so it must not
   block, must not allocate, and must not touch the ADC or flash - long
   jobs get queued and run from the main loop instead. */
void http_dispatch(const char *method, const char *path, const char *query,
                   char *scratch, uint32_t scratch_len, http_response *out);

void httpd_init(uint16_t port);
bool httpd_running(void);
uint32_t httpd_requests(void);

/* Query-string helper: copies the value of `key` into `out`, returns false
   if the key is absent.  Percent-escapes and '+' are decoded. */
bool http_query_get(const char *query, const char *key, char *out, uint32_t outsz);
long http_query_int(const char *query, const char *key, long fallback);

#ifdef __cplusplus
}
#endif

#endif /* _HTTPD_H */
