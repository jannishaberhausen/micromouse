#ifndef SHARP_H
#define	SHARP_H

#ifdef	__cplusplus
extern "C" {
#endif

void startSharp();
void sharpRaw(int *left, int *front, int *right);
void sharpDistance(int *left, int *front, int *right);
void get_walls(int *left, int *front, int *right);


#ifdef	__cplusplus
}
#endif

#endif	/* SHARP_H */

