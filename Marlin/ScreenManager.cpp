// 
// 
// 

#include "ScreenManager.h"



ScreenManagerClass ScreenManager;

//-----------------------------------------------------------------------------------------------------------------
DisplayField::DisplayField()
	{
		 x  = 0;
		 y = 0;
		 width = 1;
		 height = 1;
		 selected = false;
	}

//-----------------------------------------------------------------------------------------------------------------
DisplayFieldText::DisplayFieldText()
	{
		value = "";
		font = FONT_NORMAL;
		alignment = ALIGN_LEFT;
	}

//-----------------------------------------------------------------------------------------------------------------
Screen::Screen()
	{
		fields = NULL;
	}

//-----------------------------------------------------------------------------------------------------------------
Screen::~Screen()
	{
		if (fields)  delete fields;
		fields = NULL;
	}

//-----------------------------------------------------------------------------------------------------------------
ScreenManagerClass::ScreenManagerClass()
	{
	ScreenList = NULL;
	screenCount=0;
	current_screen=0;
	}

//-----------------------------------------------------------------------------------------------------------------
ScreenManagerClass::~ScreenManagerClass()
	{
	if (ScreenList) delete ScreenList;
	ScreenList = NULL;
	screenCount=0;
	current_screen=0;
	}
