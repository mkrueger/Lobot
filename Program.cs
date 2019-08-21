using System;
namespace LobotControl
{
	public class Program
	{
		public static void Main(string[] args)
		{
			using (var ctl = new ServoCom("/dev/ttyUSB0"))
			{
				Console.WriteLine("let ctl: " + ctl.GetLEDCtrl(1));
				Console.WriteLine("Error state:" + ctl.GetLEDError(1));
				Console.WriteLine("Pos:" + ctl.GetPosition(1) + " load or unload:" + ctl.GetLoadOrUnload(1));
				Console.WriteLine("vIn: " + ctl.GetVin(1) + " limit:" + ctl.GetVinLimit(1));
				Console.WriteLine("angle:" + ctl.GetAngleOffset(1) + " limit:" + ctl.GetAngleLimit(1));
				Console.WriteLine("temp:" + ctl.GetTemperature(1) + " max. limit:" + ctl.GetMaximumTemperature(1));


				ctl.SetMoveTime(1, 100 << 8, 0);
				Console.ReadLine();
				ctl.SetMoveTime(1, 0, 0);
				Console.ReadLine();
				ctl.SetMode(1, ServoMode.MotorControl, 200);
				Console.ReadLine();
				ctl.SetMode(1, ServoMode.PositionControl, 0);
				ctl.Stop(1);
			}
		}
	}
}
