#include <curses.h>
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <unistd.h>

int main(void){
	initscr();
	cbreak();
	noecho();
	timeout(0);
	
	char c;
	while(1){
		printf("a\n");
		sleep(1);
		if((c = getch()) != ERR){
			if(c == 's'){
				endwin();
				return 0;
			}
		}
	}
}
