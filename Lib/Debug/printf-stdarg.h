#ifndef __PRINTF_ATDARH_H_
#define __PRINTF_ATDARH_H_

/*
 * printf() and sprintf() from printf-stdarg.c
 */

int debug_printf(const char *format, ...);
int debug_sprintf(char *out, const char *format, ...);

#endif /* __PRINTF_ATDARH_H_ */