using System;
using System.IO;
using System.IO.Ports;

namespace LobotControl
{
	public class ServoCom : IDisposable
	{
		const byte HeaderByte = 0x55;

		SerialPort serialPort = new SerialPort();

		public ServoCom(string portName)
		{
			if (portName is null)
				throw new ArgumentNullException(nameof(portName));

			serialPort.BaudRate = 115200;
			serialPort.PortName = portName;
			serialPort.ReadTimeout = 1000;
			serialPort.Open();
		}

		/// <summary>
		/// Read the angle and time value which sent by SERVO_MOVE_TIME_WRITE t o the servo
		/// </summary>
		public (short angle, short time) GetMoveTime(byte id)
		{
			PrepareBuffer(SERVO_MOVE_TIME_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_MOVE_TIME_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var lowAngle = buffer[5];
			var highAngle = buffer[6];
			var lowTime = buffer[7];
			var highTime = buffer[8];

			return ((short)(highAngle << 8 | lowAngle), (short)(highTime << 8 | lowTime));
		}

		public void SetMoveTime(byte id, short angle, short time)
		{
			if (time < 0 || time > 30000)
				throw new ArgumentOutOfRangeException(nameof(time), "the range of time is 0~30000ms.");
			PrepareBuffer(SERVO_MOVE_TIME_WRITE, 10, id);
			buffer[5] = (byte)angle;
			byte highAngleValue = (byte)(angle >> 8);
			if (highAngleValue > 100) // Note: also affects negative angles !
				throw new ArgumentOutOfRangeException(nameof(angle), "higher 8 bits of angle value.range 0~100.");
			buffer[6] = highAngleValue;
			buffer[7] = (byte)time;
			buffer[8] = (byte)(time >> 8);
			SendCommand();
		}

		/// <summary>
		/// The function of this command is similar to this “SERVO_MOVE_TIME_WRITE” command in the first point. But the difference is that the servo will not immediately turn when the command arrives at the servo,the servo will be rotated from current angle to parameter angle at unifor m speed within parameter time until the command name SERVO_MOVE_ST ART sent to servo(command value of 11)
		/// , then the servo will be rotated from current angle to setting angle at uniform speed within setting time
		/// </summary>
		public (short angle, short time) GetMoveTimeWait(byte id)
		{
			PrepareBuffer(SERVO_MOVE_TIME_WAIT_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_MOVE_TIME_WAIT_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var lowAngle = buffer[5];
			var highAngle = buffer[6];
			var lowTime = buffer[7];
			var highTime = buffer[8];

			return ((short)(highAngle << 8 | lowAngle), (short)(highTime << 8 | lowTime));
		}

		/// <summary>
		/// The function of this command is similar to this “SERVO_MOVE_TIME_WRITE” command in the first point. But the difference is that the servo will not immediately turn when the command arrives at the servo,the servo will be rotated from current angle to parameter angle at unifor m speed within parameter time until the command name SERVO_MOVE_ST ART sent to servo(command value of 11)
		/// , then the servo will be rotated from current angle to setting angle at uniform speed within setting time
		/// </summary>
		public void SetMoveTimeWait(byte id, short angle, short time)
		{
			if (time < 0 || time > 30000)
				throw new ArgumentOutOfRangeException(nameof(time), "the range of time is 0~30000ms.");
			PrepareBuffer(SERVO_MOVE_TIME_WAIT_WRITE, 10, id);
			buffer[5] = (byte)angle;
			byte highAngleValue = (byte)(angle >> 8);
			if (highAngleValue > 100) // Note: also affects negative angles !
				throw new ArgumentOutOfRangeException(nameof(angle), "higher 8 bits of angle value.range 0~100.");
			buffer[6] = highAngleValue;
			buffer[7] = (byte)time;
			buffer[8] = (byte)(time >> 8);
			SendCommand();
		}

		/// <summary>
		/// Description: ID read is a little bit special compared with other read commands , if the command packet ID is broadcast ID:254 (0xFE), the servo will return the response information, and other read commands will not return the response message when ID is broadcast ID. The purpose of this design is to inquiry the servo ID number via broadcast ID without knowing the ID number of the servo, but the limit is that the bus can only access a servo, or it will return data caused bus conflict.
		/// </summary>
		public short GetId(byte id)
		{
			PrepareBuffer(SERVO_ID_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_ID_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var idResult = buffer[5];
			return (byte)idResult;
		}

		public void SetId(byte id, byte newId)
		{
			if (newId > 253)
				throw new ArgumentOutOfRangeException(nameof(newId), "The servo ID, range 0 ~ 253, defaults to 1.");
			PrepareBuffer(SERVO_ID_WRITE, 7, id);
			buffer[5] = newId;
			SendCommand();
		}

		/// <summary>
		/// With the use of command SERVO_MOVE_TIME_WAIT_WRITE
		/// </summary>
		public void Start(byte id)
		{
			PrepareBuffer(SERVO_MOVE_START, 6, id);
			SendCommand();
		}

		/// <summary>
		/// When the command arrives at the servo, it will stop running immediately if the servo is rotating, and stop at the current angle position.
		/// </summary>
		public void Stop(byte id)
		{
			PrepareBuffer(SERVO_MOVE_STOP, 6, id);
			SendCommand();
		}

		public sbyte GetAngleOffset(byte id)
		{
			PrepareBuffer(SERVO_ANGLE_OFFSET_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_ANGLE_OFFSET_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var idResult = buffer[5];
			return (sbyte)idResult;
		}

		/// <summary>
		/// Save the deviation value, and support “power-down save”.
		/// </summary>
		public void WriteAngleOffset(byte id)
		{
			PrepareBuffer(SERVO_ANGLE_OFFSET_WRITE, 6, id);
			SendCommand();
		}

		/// <summary>
		/// Note 1:The adjusted deviation value is not saved when power-down by this command, if you want to save please refer to point 10.
		/// Note 2: Because the parameter is “signed char” type of data, and the command packets to be sent are “unsigned char” type of data, so before sending, parameters are forcibly converted to “unsigned char” data and then put them in command packet.
		/// </summary>
		/// <param name="servoDeviation">range -125~ 125, The corresponding angle of -30 ° ~ 30 °, when this command reach to the servo, the servo will immediately rotate to adjust the deviation.</param>
		public void AdjustAngleOffset(byte id, sbyte servoDeviation)
		{
			PrepareBuffer(SERVO_ANGLE_OFFSET_ADJUST, 7, id);
			buffer[5] = (byte)servoDeviation;
			SendCommand();
		}

		public (short min, short max) GetAngleLimit(byte id)
		{
			PrepareBuffer(SERVO_ANGLE_LIMIT_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_ANGLE_LIMIT_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var lowMin = buffer[5];
			var highMin = buffer[6];
			var lowMax = buffer[7];
			var highMax = buffer[8];

			return ((short)(highMin << 8 | lowMin), (short)(highMax << 8 | lowMax));
		}

		public void SetAngleLimit(byte id, short min, short max)
		{
			PrepareBuffer(SERVO_ANGLE_LIMIT_WRITE, 10, id);
			buffer[5] = (byte)min;
			byte highAngleValue = (byte)(min >> 8);
			if (highAngleValue > 100) // Note: also affects negative angles !
				throw new ArgumentOutOfRangeException(nameof(min), "higher 8 bits of angle value.range 0~100.");
			buffer[6] = highAngleValue;
			buffer[7] = (byte)max;
			highAngleValue = (byte)(max >> 8);
			if (highAngleValue > 100) // Note: also affects negative angles !
				throw new ArgumentOutOfRangeException(nameof(min), "higher 8 bits of angle value.range 0~100.");
			buffer[8] = (byte)(highAngleValue);
			SendCommand();
		}

		public (short vMin, short vMax) GetVinLimit(byte id)
		{
			PrepareBuffer(SERVO_VIN_LIMIT_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_VIN_LIMIT_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var lowMin = buffer[5];
			var highMin = buffer[6];
			var lowMax = buffer[7];
			var highMax = buffer[8];

			return ((short)(highMin << 8 | lowMin), (short)(highMax << 8 | lowMax));
		}

		public void SetVinLimit(byte id, short vMin, short vMax)
		{
			if (vMin < 4500 || vMin > 12000)
				throw new ArgumentOutOfRangeException(nameof(vMin), "range 4500~12000mv");

			if (vMax < 4500 || vMax > 12000)
				throw new ArgumentOutOfRangeException(nameof(vMax), "range 4500~12000mv");

			PrepareBuffer(SERVO_VIN_LIMIT_WRITE, 10, id);
			buffer[5] = (byte)vMin;
			buffer[6] = (byte)(vMin >> 8);
			buffer[7] = (byte)vMax;
			buffer[8] = (byte)(vMax >> 8);
			SendCommand();
		}

		public int GetMaximumTemperature(byte id)
		{
			PrepareBuffer(SERVO_TEMP_MAX_LIMIT_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_TEMP_MAX_LIMIT_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var max = buffer[5];
			return max;
		}

		public void SetMaximumTemperature(byte id, int temp)
		{
			if (temp < 50 || temp > 100)
				throw new ArgumentOutOfRangeException(nameof(temp), "The maximum temperature limit inside the servo range 50~100°C.");

			PrepareBuffer(SERVO_TEMP_MAX_LIMIT_WRITE, 7, id);
			buffer[5] = (byte)temp;
			SendCommand();
		}

		public byte GetTemperature(byte id)
		{
			PrepareBuffer(SERVO_TEMP_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_TEMP_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var currentTemperature = buffer[5];
			return (byte)currentTemperature;
		}

		public short GetVin(byte id)
		{
			PrepareBuffer(SERVO_VIN_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_VIN_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var lowCurrentInputVoltage = buffer[5];
			var highCurrentInputVoltage = buffer[6];
			return (short)(highCurrentInputVoltage << 8 | lowCurrentInputVoltage);
		}

		public short GetPosition(byte id)
		{
			PrepareBuffer(SERVO_POS_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_POS_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);

			var low = buffer[5];
			var high = buffer[6];

			return (short)(high << 8 | low);
		}

		public (ServoMode mode, short rotationSpeed) GetMode(byte id)
		{
			PrepareBuffer(SERVO_OR_MOTOR_MODE_READ, 6, id);
			SendCommand();

			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_OR_MOTOR_MODE_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);


			var mode = buffer[5];
			var nullByte = buffer[6];
			if (nullByte != 0)
				throw new InvalidOperationException("nullByte != 0 : " + nullByte);
			var lowRotSpeed = buffer[7];
			var highRotSpeed = buffer[8];

			return ((ServoMode)mode, (short)(highRotSpeed << 8 | lowRotSpeed));
		}

		public void SetMode(byte id, ServoMode mode, short rotationSpeed)
		{
			// TODO : may be wrong. may be high rotation speed byte -100 to 100.
			if (rotationSpeed < -1000 || rotationSpeed > 1000)
				throw new ArgumentOutOfRangeException(nameof(rotationSpeed));
			PrepareBuffer(SERVO_OR_MOTOR_MODE_WRITE, 8, id);
			buffer[5] = (byte)mode;
			buffer[6] = 0; // null value
			buffer[7] = (byte)rotationSpeed;
			byte highRotationSpeed = (byte)(rotationSpeed >> 8);
			buffer[6] = highRotationSpeed;
			SendCommand();
		}

		public bool GetLoadOrUnload(byte id)
		{
			PrepareBuffer(SERVO_LOAD_OR_UNLOAD_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_LOAD_OR_UNLOAD_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);
			return buffer[5] == 0;
		}

		public void SetLoadOrUnload(byte id, bool isUnloaded)
		{
			PrepareBuffer(SERVO_LOAD_OR_UNLOAD_WRITE, 7, id);
			buffer[5] = (byte)(isUnloaded ? 0 : 1);
			SendCommand();
		}

		public bool GetLEDCtrl(byte id)
		{
			PrepareBuffer(SERVO_LED_CTRL_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_LED_CTRL_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);
			return buffer[5] == 0;
		}

		public void SetLEDCtrl(byte id, bool state)
		{
			PrepareBuffer(SERVO_LED_CTRL_WRITE, 7, id);
			buffer[5] = (byte)(state ? 0 : 1);
			SendCommand();
		}

		public ErrorState GetLEDError(byte id)
		{
			PrepareBuffer(SERVO_LED_ERROR_READ, 6, id);
			SendCommand();
			ReceivePackage();
			if (id != buffer[2])
				throw new InvalidProgramException("id != response id");
			if (buffer[4] != SERVO_LED_ERROR_READ)
				throw new InvalidProgramException("wrong response command: " + buffer[4]);
			return (ErrorState)buffer[5];
		}

		void ReceivePackage()
		{
			if (serialPort.ReadByte() != HeaderByte ||
				serialPort.ReadByte() != HeaderByte)
				throw new InvalidDataException("Header mismatch");
			buffer[2] = (byte)serialPort.ReadByte();
			buffer[3] = (byte)serialPort.ReadByte();
			serialPort.Read(buffer, 4, buffer[3] - 1);
			if (CalcChecksum(buffer) != buffer[buffer[3] + 2])
				throw new InvalidDataException("Checksum doesn't match");
		}

		public void SetLEDError(byte id, ErrorState state)
		{
			if ((int)state > 7)
				throw new ArgumentOutOfRangeException(nameof(state));
			PrepareBuffer(SERVO_LED_ERROR_WRITE, 7, id);
			buffer[5] = (byte)state;
			SendCommand();
		}

		byte[] buffer;

		byte[] PrepareBuffer(byte command, byte length, byte id)
		{
			if (buffer == null || length > buffer.Length)
			{
				buffer = new byte[Math.Max((int)length, 10)]; // 10 is max atm
				buffer[0] = buffer[1] = HeaderByte;
			}
			buffer[2] = id;
			buffer[3] = (byte)(length - 3);
			buffer[4] = command;

			return buffer;
		}

		void SendCommand()
		{
			int length = buffer[3] + 3;
			buffer[length - 1] = CalcChecksum(buffer);
			serialPort.Write(buffer, 0, length);
		}

		static byte CalcChecksum(byte[] buffer)
		{
			unchecked
			{
				byte res = 0;
				for (var i = buffer[3] + 1; i >= 2; i--)
				{
					res += buffer[i];
				}
				return (byte)~res;
			}
		}

		public void Dispose()
		{
			serialPort?.Dispose();
			serialPort = null;
		}

		#region constants
		const byte SERVO_MOVE_TIME_WRITE = 1;
		const byte SERVO_MOVE_TIME_READ = 2;
		const byte SERVO_MOVE_TIME_WAIT_WRITE = 7;
		const byte SERVO_MOVE_TIME_WAIT_READ = 8;
		const byte SERVO_MOVE_START = 11;
		const byte SERVO_MOVE_STOP = 12;
		const byte SERVO_ID_WRITE = 13;
		const byte SERVO_ID_READ = 14;
		const byte SERVO_ANGLE_OFFSET_ADJUST = 17;
		const byte SERVO_ANGLE_OFFSET_WRITE = 18;
		const byte SERVO_ANGLE_OFFSET_READ = 19;
		const byte SERVO_ANGLE_LIMIT_WRITE = 20;
		const byte SERVO_ANGLE_LIMIT_READ = 21;
		const byte SERVO_VIN_LIMIT_WRITE = 22;
		const byte SERVO_VIN_LIMIT_READ = 23;
		const byte SERVO_TEMP_MAX_LIMIT_WRITE = 24;
		const byte SERVO_TEMP_MAX_LIMIT_READ = 25;
		const byte SERVO_TEMP_READ = 26;
		const byte SERVO_VIN_READ = 27;
		const byte SERVO_POS_READ = 28;
		const byte SERVO_OR_MOTOR_MODE_WRITE = 29;
		const byte SERVO_OR_MOTOR_MODE_READ = 30;
		const byte SERVO_LOAD_OR_UNLOAD_WRITE = 31;
		const byte SERVO_LOAD_OR_UNLOAD_READ = 32;
		const byte SERVO_LED_CTRL_WRITE = 33;
		const byte SERVO_LED_CTRL_READ = 34;
		const byte SERVO_LED_ERROR_WRITE = 35;
		const byte SERVO_LED_ERROR_READ = 36;
		#endregion
	}
}
