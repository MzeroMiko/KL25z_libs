#include <stdarg.h>
#include "redrivers/uart.h"

#define MAX_CMD_NUM 5
#define MAX_ARG_NUM 10
#define MAX_ARG_STR 20

#define intarg(arg) atoi(arg)
#define chararg(arg) (*((char *)arg))
#define strarg(arg) ((char *)arg)

typedef unsigned int uint;
typedef unsigned char uchar;

typedef struct {
    int (*func)(void *args[]), argsnum; 
    char *name, *discription; 
} shell_cmdcontext;

void *memset(void *dest, int8_t val, uint32_t count);
char *strcpy(char *s, char *t);
int strcmp(char *p, char *q);
uint strlen(const char *s);
char *strchr(char *s, char c);
int strindex(char *s, char c);
int atoi(char *s);
char *itoa(int xx, int base, int sgn, char *buf);

void shell_printint(int xx, int base, int sgn);
void shell_printf(const char *fmt, ...);
void shell_getline(char *s);
void shell_registercmd(char *name, int func(void *args[]), int argsnum, char *discription);
void shell_unregistercmd(char *name);
void shell_main(void);

