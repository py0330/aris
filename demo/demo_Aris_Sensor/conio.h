#ifndef CONIO_H
#define CONIO_H

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif

EXTERN_C int _getch(void);
EXTERN_C int _kbhit(void);

#endif
