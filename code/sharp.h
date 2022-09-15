#ifndef SHARP_H
#define	SHARP_H

#define SHARP_MAX 4095

#ifdef	__cplusplus
extern "C" {
#endif

void initSharp();
void sharpRaw(int *left, int *front, int *right);
void sharpDistance(int *left, int *front, int *right);
void get_walls(int *left, int *front, int *right);


#ifdef	__cplusplus
}
#endif

#endif	/* SHARP_H */

