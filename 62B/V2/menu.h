#define LCD_LEFT 1
#define LCD_CENTER 2
#define LCD_LC 3
#define LCD_RIGHT 4
#define LCD_LR 5
#define LCD_CR 6
#define LCD_ALL 7

bool picked = false;

int auto = 0;

task ProgramChooser()
{

	bLCDBacklight = true;
	clearLCDLine(0);
	clearLCDLine(1);

	int max = 3;
	int min = 0;

	while(!picked)
	{
		if(nLCDButtons == LCD_LEFT)
		{
			while(nLCDButtons != 0) {}
			auto--;
		}
		if(nLCDButtons == LCD_RIGHT)
		{
			while(nLCDButtons != 0) {}
			auto++;
		}
		if(nLCDButtons == LCD_CENTER)
		{
			while(nLCDButtons != 0) {}
			picked = true;
		}

		if(auto < min) auto = max;
		if(auto > max) auto = min;
		string autoName = "";
		switch(auto)
		{
		case 0: autoName = "[SKYRISE][BLUE]"; break;
		case 1: autoName = "[NORISE][BLUE]"; break;
		case 2: autoName = "[SKYRISE][RED]"; break;
		case 3: autoName = "[NORISE][RED]"; break;
		}
		displayLCDCenteredString(0,autoName);
		displayLCDCenteredString(1,"<            >");
	}
}
