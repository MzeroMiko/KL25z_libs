#include "sh.h"

int shell_cmdnum = 0;
shell_cmdcontext shell_cmdlist[MAX_CMD_NUM];

void *memset(void *dest, int8_t val, uint32_t count) {
    int8_t *temp = (int8_t *)dest;
    for( ; count != 0; count--) *temp++ = val;
    return dest;
}

char *strcpy(char *s, char *t) {
    char *os;
    os = s;
    while((*s++ = *t++) != 0);
    return os;
}

int strcmp(char *p, char *q) {
    while(*p && *p == *q) p++, q++;
    return (uchar)*p - (uchar)*q;
}

uint strlen(const char *s) {
    int n;
    for(n = 0; s[n]; n++);
    return n;
}

char *strchr(char *s, char c) {
    for(; *s; s++)
        if(*s == c) return (char*)s;
    return 0;
}

int strindex(char *s, char c) {
    int i=0;
    for(; *s; s++, i++)
        if(*s == c) return i;
    return -1;
}

int atoi(char *s) {
    int n = 0, neg = 0;
    if (*s == '-') neg = 1, s++;
    while('0' <= *s && *s <= '9') n = n * 10 + *(s++) - '0';
    return neg? -n : n;
}

char *itoa(int xx, int base, int sgn, char *buf) {
    static char digits[] = "0123456789ABCDEF";
    char *obuf = buf;
    char tbuf[16]; // uint32_t max is 4294967296
    int i = 0, neg = (sgn && xx < 0);
    uint x = neg ? (uint)(-xx) : (uint)(xx);
    do {
        tbuf[i++] = digits[x % base];
    } while((x /= base) != 0);
    if(neg) *(buf++) = '-';
    while(--i >= 0) *(buf++) = tbuf[i];
    *buf = '\0';
    return obuf;
}


void shell_putc(char c) {
    UART0_SendByte((uint8_t) c);
}

char shell_getc(void) {
    return (char)(UART0_RecvByte());
}

void shell_printint(int xx, int base, int sgn) {
    static char digits[] = "0123456789ABCDEF";
    char buf[16]; // uint32_t max is 4294967296
    int i = 0, neg = (sgn && xx < 0);
    uint x = neg ? (uint)(-xx) : (uint)(xx);

    do {
        buf[i++] = digits[x % base];
    } while((x /= base) != 0);

    if(neg) shell_putc('-');
    while(--i >= 0) shell_putc(buf[i]);
}

void shell_printf(const char *fmt, ...) {
    char *s;
    int i = 0;
    char c = 0, state = 0;
    va_list ap;
    va_start(ap, fmt);
    for (i = 0; fmt[i] != '\0'; i++) {
        c = fmt[i] & 0xff;
        if (state == '%') {
            state = 0;
            switch (c) {
                case 'd':
                    shell_printint(va_arg(ap, int), 10, 1);
                    break;
                case 'x': case'p':
                    shell_printint(va_arg(ap, int), 16, 0);
                    break;
                case 's':
                    s = va_arg(ap, char *);
                    if (s == 0) break;
                    while(*s != 0)  shell_putc(*(s++));
                    break;
                case 'c':
                    shell_putc(va_arg(ap, int));
                    break;
                case '%':
                    shell_putc(c);
                    break;
                default:
                    shell_putc('%');
                    shell_putc(c);
            }
        } else {
            if (c == '%') state = '%';
            else shell_putc(c);
        }
    }
    va_end(ap);
}

void shell_getline(char *s) {
    while( (*(s++) = shell_getc()) != '\n');
}

void shell_getargs(int *argc, char *argv[]) {
    char recv_data;
    int i = 0;
    *argc = 0;
    while((recv_data = shell_getc()) != '\n') {
        if (recv_data == ' ') {
            if (i != 0) {
                argv[*argc][i] = '\0'; 
                (*argc)++;
                i = 0;
            } 
        } else {
            argv[*argc][i++] = recv_data;
        }
    }
    if (i != 0) { argv[*argc][i] = '\0'; (*argc)++; }
}

void shell_registercmd(char *name, int func(void *args[]), int argsnum, char *discription) {
    shell_cmdlist[shell_cmdnum].name = name; 
    shell_cmdlist[shell_cmdnum].func = func; 
    shell_cmdlist[shell_cmdnum].argsnum = argsnum; 
    shell_cmdlist[shell_cmdnum].discription = discription; 
    shell_cmdnum++;
}

void shell_unregistercmd(char *name) {
    int i = 0, j = 0;
    shell_cmdcontext *tmp;
    for (i = 0; i < MAX_CMD_NUM; i++ ) {
        if (!strcmp(shell_cmdlist[i].name, name)) {
            tmp = &(shell_cmdlist[i]);
            for (j = i; j < MAX_CMD_NUM - 1; j++)
                shell_cmdlist[j] = shell_cmdlist[j+1]; 
            shell_cmdlist[MAX_CMD_NUM - 1] = *tmp;
            shell_cmdnum--;
            break;
        }   
    }
}

void shell_ls(void) {
    int i = 0;
    shell_cmdcontext *tmp;
    shell_printf("Buildin CMD: ls > ls all cmds\n");
    shell_printf("Buildin CMD: exit > exit this shell\n");
    for (i = 0; i < shell_cmdnum; i++ ) {
        tmp = &(shell_cmdlist[i]);
        shell_printf("Registered CMD %d: %s > %s\n", i, tmp->name, tmp->discription);
    }
}

void shell_main(void) {
    int i, argc, reval;
    shell_cmdcontext *tmp;
    char _argv[MAX_ARG_NUM][MAX_ARG_STR];
    char *argv[MAX_ARG_NUM];
    for (argc = 0; argc < MAX_ARG_NUM; argc ++) argv[argc] = &(_argv[argc][0]);

    UART0_Init(9600);
    shell_printf("Simple Shell for KL25Z by Mzero.\n");
    while(1) {
        argc = 0;
        shell_printf("sh:");
        while(argc == 0) shell_getargs(&argc, argv);
        // for (i = 0; i < argc; i++) shell_printf("%d:%s\n", i, argv[i]);
        
        if (!strcmp("ls", argv[0])) shell_ls();
        else if (!strcmp("exit", argv[0])) { shell_printf("Warn: shell exited.\n"); break; }
        else {
            for (i = 0; i < shell_cmdnum; i++ ) {
                tmp = &(shell_cmdlist[i]);
                if (!strcmp(tmp->name, argv[0])) {
                    if (argc == tmp->argsnum + 1) {
                        reval = (tmp->func)((void **)(argv + 1));
                        if (reval) shell_printf("CMD exist with code %d\n", reval);
                    }
                    else shell_printf("Error: %s expected %d args, but got %d.\n", tmp->name, tmp->argsnum, argc - 1);
                    break;
                }
            }
            if (i == shell_cmdnum) shell_printf("Error: cmd not found.\n");
        }
    }
}

