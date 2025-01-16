/*
 * dino.h
 *
 *  Created on: Jan 13, 2025
 *      Author: esthe
 */

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

typedef struct {
	int jump;
	int score;
} Dino;

typedef struct {
	int locations[20];
	int spaces; // how many spaces after the last cactus
	int cacti; // for double cacti
} Cactus;

typedef enum {
	menu,
	in_progress,
	over
} GameStatus;

void cactus_update (Cactus * cactus);
void init (Cactus * cactus, Dino * dino, GameStatus * gamestatus);
void display(Cactus * cactus, Dino * dino);
void collision(Cactus * cactus, Dino * dino, GameStatus * gamestatus);
void clear_screen();
