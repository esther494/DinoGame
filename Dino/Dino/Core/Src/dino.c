/*
 * dino.c
 *
 *  Created on: Jan 13, 2025
 *      Author: esthe
 */

#include "dino.h"
#include "i2c-lcd.h"

void cactus_update (Cactus * cactus) {
	// We will move the cacti to the left by one
	for (int i = 0; i < 19; i++) {
		cactus->locations[i] = cactus->locations[i+1];
	}

	// cactus spawning have to have at least 3 spaces for the player
	// added a feature that will spawn 2 cacti
	if (cactus->cacti == 1) {
		cactus->locations[19] = 1;
		cactus->cacti = 0;
	}
	else {
		if (cactus->spaces > 2) {
			if (rand() % 2 == 1) {
				cactus->spaces = 0;
				cactus->locations[19] = 1;
				if (rand() % 5 == 1) cactus->cacti = 1; // determine if this will be a double
				// the change will be in effect when the function is called again (next timer cycle)
			}
		}
		else {
			cactus->spaces++;
			cactus->locations[19] = 0;
		}
	}
}

void init (Cactus * cactus, Dino * dino, GameStatus * gamestatus) {
	for (int i = 0; i < 20; i++) {
		cactus->locations[i] = 0;
	}
	cactus->spaces = 0;
	cactus->cacti = 0;

	dino->score = 0;
	dino->jump = 0;
}

void display(Cactus * cactus, Dino * dino) {
	clear_screen();

	if (dino->jump >= 1) {
		lcd_cursor_pos(1, 4);
		lcd_send_data(0);
	}
	else {
		lcd_cursor_pos(3, 4);
		lcd_send_data(0);
	}

	for (int i = 0; i < 20; i++) {
		if (cactus->locations[i] == 1) {
			lcd_cursor_pos(3, i);
			lcd_send_data(1);
		}
	}
}

void collision(Cactus * cactus, Dino * dino, GameStatus * gamestatus) {
	if (cactus->locations[4] == 1 && dino->jump == 0) {
		*gamestatus = over;
	}
}

void clear_screen() {
	lcd_clear_display();
	// for some reason, my lcd won't clear the bottom right part of the screen
	// so i have to manually send a blank space to clear them
	for (int col = 9; col <= 19; col++) {
		lcd_cursor_pos(3, col);
		lcd_send_data(' ');
	}
}
