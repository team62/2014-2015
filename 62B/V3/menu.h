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
		case 0: autoName = "Blue Skyrise"; break;
		case 1: autoName = "Blue Norise"; break;
		case 2: autoName = "Red Skyrise"; break;
		case 3: autoName = "Red Norise"; break;
		}
		displayLCDCenteredString(0,autoName);
		displayLCDCenteredString(1,"<            >");
	}
}
