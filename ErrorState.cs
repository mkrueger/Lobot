namespace LobotControl
{
	public enum ErrorState : byte
	{
		NoAlarm = 0,
		OverTemperature = 1,
		OverVoltage = 2,
		OverTemperatureAndVoltage = 3,
		LockedRotor = 4,
		OverTemperatureAndStalled = 5,
		OverVoltageAndStalled = 6,
		OverTemperatureOverVolatageAndStalled = 7
	}
}
