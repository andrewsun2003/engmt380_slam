================================================================================
====================== SLAMBotHD control module Usage ==========================
================================================================================

Variables
	imuYaw, imuPitch & imuRoll	{in degrees}
	imuAngVel			{in rad/s (about Z-axis)}
	encLeft, encRight		{2578.33 ticks/rev}
	botBatt				{Battery voltage in V}
	imgDepth			{Depth Image}
	imgColor			{Color Image}
	bumperSide, bumperHit		{Side = LEFT:0, CENTER:1, RIGHT:2}
					{Hit = RELEASED:0, PRESSED:1}
	wheelSide, wheelLift		{Side = LEFT:0, RIGHT:1}
					{Lift = RAISED:0, DROPPED:1}	
	buttonNumber,buttonState	{Num = B0:0, B1:1, B2:2}
					{State = RELEASED:0, PRESSED:1}			

Methods:
	***************** MANDATORY CALLS *****************
	startUp()			MANDATORY: Must be called at start
	shutDown()			MANDATORY: Must be called at end
	
	******************* OTHER CALLS *******************
	botInactive()			Checks if bot comms active
					Return Value
						0:Comms Active
						1:Comms Inactive
	readCoreData()			Updates all readable variables
	moveBot(linVel, angVel)	Moves bot with user specified parameters
					linVel - linear velocity (min 0, max 0.6)
					angVel - angular velocity (min 0, max 1.7)
	changeLeds(l1_Val, l2_Val)	Lights up bot's LEDs {latching function}
					l1_Val - color value for LED1
					l2_Val - color value for LED2
					colors - 0:OFF, 1:GREEN, 2:AMBER, 3:RED
	playSound(Tone_ID)		Plays sounds {Tone selection below}
					0:ON, 1:OFF, 2:RECHARGE START, 3: BEEP
					4:ERROR_TONE, 5: STARTUP, 6: FINISH
