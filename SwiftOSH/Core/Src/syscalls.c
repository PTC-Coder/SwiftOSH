/**
  * @file    syscalls.c
  * @brief   Minimal newlib syscall stubs for bare-metal (suppresses linker warnings)
  */

#include <sys/stat.h>
#include <errno.h>

int _close(int fd)    { (void)fd; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _getpid(void)     { return 1; }
int _isatty(int fd)   { (void)fd; return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _lseek(int fd, int offset, int whence) { (void)fd; (void)offset; (void)whence; return 0; }
int _read(int fd, char *buf, int len) { (void)fd; (void)buf; (void)len; return 0; }
int _write(int fd, char *buf, int len) { (void)fd; (void)buf; return len; }
