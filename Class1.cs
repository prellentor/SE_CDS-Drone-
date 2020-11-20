using System;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using VRageMath;
using VRage.Game;
using Sandbox.ModAPI.Interfaces;
using Sandbox.ModAPI.Ingame;
using Sandbox.Game.EntityComponents;
using VRage.Game.Components;
using VRage.Collections;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game.ModAPI.Ingame;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage.Game.GUI.TextPanel;

namespace CDS_Drone_
{
  public sealed class Program : MyGridProgram
  {

    #region "Program"
    public static Program ProgramInstance;
    public static readonly float MAX_STEER_ANGLE = (float)(Math.PI / 4.0);
    class SuspensionWrapper
    {
      public IMyMotorSuspension Suspension;
      public int Side = 1;
      public IMyShipController Controller;
      private float ToGridCenterDistance = 0;

      public SuspensionWrapper(IMyMotorSuspension newSuspension, ref IMyShipController controller)
      {
        Suspension = newSuspension;
        Controller = controller;
        ToGridCenterDistance = (float)(Controller.CubeGrid.GetPosition() - Suspension.GetPosition()).Length();
        Suspension.MaxSteerAngle = (float)(Math.PI / 4.0);
      }

      public void SuspensionStrengthChanged(object sender, SuspensionStrengthChangedEventArgs e)
      {
        //float newStrength = e.Strength * (float)
          //(Controller.CenterOfMass - Suspension.GetPosition()).Length() / ToGridCenterDistance;
        //if (Math.Round(Suspension.Strength, 1) != Math.Round(newStrength, 1))
          Suspension.Strength = e.Strength;//newStrength;
      }

      public void SuspensionParametersChanged(object sender, SuspensionParametersChangedEventArgs e)
      {
        Suspension.SetValueFloat("Speed Limit", e.Speed);
        Suspension.Friction = e.Friction;
        Suspension.SetValueFloat("Steer override", e.Steer);
        Suspension.SetValueFloat("Propulsion override", e.Movement * Side);
      }
    }

    public delegate void SuspensionParametersChangedHandler(object sender, SuspensionParametersChangedEventArgs e);

    public delegate void SuspensionStrengthChangedHandler(object sender, SuspensionStrengthChangedEventArgs e);

    public class SuspensionStrengthChangedEventArgs : EventArgs
    {
      public float Strength { get; set; }

      public SuspensionStrengthChangedEventArgs() { Strength = 1.0f; }
    }

    public class SuspensionParametersChangedEventArgs : EventArgs
    {
      public float Speed { get; set; }
      public float Friction { get; set; }
      public float Steer { get; set; }
      public float Movement { get; set; }

      public SuspensionParametersChangedEventArgs()
      {
        Speed = 0.0f;
        Friction = 35.0f;
        Steer = 0.0f;
        Movement = 0.0f;
      }
    }

    MULE unit;
    public Program()
    {
      ProgramInstance = this;
      unit = new MULE();
      Runtime.UpdateFrequency = UpdateFrequency.Update1; 
    }
    #endregion

    public enum Gear { NEUTRAL, FORWARD, BACKWARD };

    public class RouteNode 
    {
      public string Name;
      public Vector3D Position;
      public float Speed;
      public double CheckDistance;
      public Gear Direction;

      public RouteNode(string Line)
      {
        //0  1   2    3    4    5
        //20:3.5:direction:name:15934.19:-15396.17:16499.03:
        //will be changed 
        //speed:radius:direction:possible evasive:radio call:name:X:Y:Z:

        string[] words = Line.Split(':');
        Speed = float.Parse(words[0]);
        CheckDistance = double.Parse(words[1]);
        Direction = (Gear) int.Parse(words[2]);
        Name = words[3];
        Position = new Vector3D(double.Parse(words[4]), double.Parse(words[5]), double.Parse(words[6]));
      }
    }

    public class MULE
    {
      #region "Fields"

      string StatusData { get { return ProgramInstance.Me.GetSurface(0).GetText().Trim(); } }
      string RouteData { 
        get { return ProgramInstance.Me.GetSurface(1).GetText().Trim(); } 
        set { ProgramInstance.Me.GetSurface(1).WriteText(value, false); }
      }

      IMyShipConnector _connector;
      IMySensorBlock _sensor;
      IMyShipController _controller;

      string
        _connectorName = "Connector Forward",
        _sensorName = "Sensor",
        _controllerName = "Remote Control",
        _previousInput = "",
        _sender = "NULL",
        _recipient = "NULL",
        _mode = "",
        _radio = "",
        _goodbayTarget = "";

      int
        _evasive,
        _routeLength,
        _currentNodeNum;

      float 
        _maxForwardSpeed = 80,
        _suspensionSoftnessFactor = 37.5f,
        _angleToTarget = 0f;

      double
        _currentSpeed,
        _distanceToTarget;

      List<SuspensionWrapper> _suspensions = new List<SuspensionWrapper>();
      event SuspensionParametersChangedHandler ChangeTruckSuspensionParameters;
      event SuspensionStrengthChangedHandler ChangeTruckSuspensionStrength;

      Vector3D 
        _rollAxle = new Vector3D(),
        _pitchAxle = new Vector3D(),
        _gridCenterPosition = new Vector3D();

      enum State { 
        STANDBY, 
        MOVE, 
        BASEMOVE, 
        CLEARCOMM, 
        CLEARROUTE, 
        READROUTE, 
        DOCKING, 
        UNDOCKING,
        RADIOCALL 
      };

      State _status = State.STANDBY;

      List<string> _commands = new List<string>();

      List<RouteNode> _route = new List<RouteNode>();

      #endregion

      #region "Data parse and storage"
      /// <summary>
      /// it could be better to store that in remote control gps list
      /// all arguments can be used in gps name
      /// name - 20,3.5,1,Node gps - 15934.19:-15396.17:16499.03
      /// no need to shit in ram
      /// </summary>
      void ReadRouteData()
      {
        var text = RouteData;
        if (string.IsNullOrWhiteSpace(text))
          return;

        _route.Clear();

        string[] lines = text.Trim().Split('\n');
        for (int l = 0; l < lines.Length; l++)
        {
          try
          {
            _route.Add(new RouteNode(lines[l]));
          }
          catch (Exception e)
          {
            ProgramInstance.Echo("Exception occured during reading route data: " + e.Message);
          }
        }
        _routeLength = _route.Count - 1;
      }

      State StatusEnumFromString(string strStatus)
      {
        switch (strStatus.ToUpper())
        {
          case "BASEMOVE":
            return State.BASEMOVE;
          case "CLEARCOMM":
            return State.CLEARCOMM;
          case "CLEARROUTE":
            return State.CLEARROUTE;
          case "DOCKING":
            return State.DOCKING;
          case "MOVE":
            return State.MOVE;
          case "READROUTE":
            return State.READROUTE;
          case "UNDOCKING":
            return State.UNDOCKING;
          case "RADIOCALL":
            return State.RADIOCALL;
        }
        return State.STANDBY;
      }

      void ReadData()
      {
        /*
         * Sender: TestStation1
         * Recipient: teststation2
         * Mode: courier|fred 
         * Radio: teststation2
         * Goodbay: DroRadInp23 9899994525615633
         * State: STANDBY
         * CurrPath: 0
         * ConnStat: Connected
         */
        var text = StatusData;
        _previousInput = text;
        string[] lines = text.Split('\n');

        string[] wordPair = lines[0].Split(':');
        _sender = string.IsNullOrWhiteSpace(wordPair[1]) ? "NULL" : wordPair[1].Trim();

        wordPair = lines[1].Split(':');
        _recipient = string.IsNullOrWhiteSpace(wordPair[1]) ? "NULL" : wordPair[1].Trim();

        wordPair = lines[2].Split(':');
        _mode = wordPair[1];

        wordPair = lines[3].Split(':');
        _radio = wordPair[1];

        wordPair = lines[4].Split(':');
        _goodbayTarget = wordPair[1];

        wordPair = lines[5].Split(':');
        _status = StatusEnumFromString(wordPair[1]);

        MoveOrStop();

        wordPair = lines[6].Split(':');
        _currentNodeNum = int.Parse(wordPair[1]);

        _commands = new List<string>();
        //7 - ConnStat
        for (int l = 8; l < lines.Length; l++)
        {
          wordPair = lines[l].Split(':');
          try
          {
            switch (wordPair[0].ToUpper())
            {
              case "NEXTCOMM":
                _commands.Add(wordPair[1].ToUpper());
                break;

              case "NEWSTATE":
                _status = StatusEnumFromString(wordPair[1]);
                break;

              case "NEWCOMMAND":
                _commands.Add(wordPair[1].ToUpper());
                break;

              case "NEWSENDER":
                _sender = wordPair[1];
                break;

              case "NEWRECIPIENT":
                _recipient = wordPair[1];
                break;

              case "NEWMODE":
                _mode = wordPair[1];
                break;

              case "NEWRADIO":
                _radio = wordPair[1];
                break;

              case "NEWGOODBAY":
                _goodbayTarget = wordPair[1];
                break;

              default:
                break;
            }
          }
          catch
          (Exception e)
          {
            ProgramInstance.Echo("At: " + (l + 1) + "\n" + e.ToString());
          }
        }//end for
      }

      void WriteData()
      {
        /*
         * Sender: TestStation1
         * Recipient: teststation2
         * Mode: courier|fred 
         * Radio: teststation2
         * Goodbay: DroRadInp23 9899994525615633
         * State: STANDBY
         * CurrPath: 0
         * ConnStat: Connected
         */
        StringBuilder sb = new StringBuilder
                 ("Sender:" + _sender + "\n");
        sb.Append("Recipient:" + _recipient + "\n");
        sb.Append("Mode:" + _mode + "\n");
        sb.Append("Radio:" + _radio + "\n");
        sb.Append("Goodbay:" + _goodbayTarget + "\n");
        sb.Append("State:" + _status + "\n");
        sb.Append("CurrPath:" + _currentNodeNum + "\n");
        sb.Append("ConnStat:" + _connector.Status.ToString() + "\n");
        foreach (string command in _commands)
          sb.Append("NextComm:" + command + "\n");
        ProgramInstance.Me.GetSurface(0).WriteText(sb.ToString(), false);
      }

      internal void NewInstructions(string argument)
      {
        string[] lines = argument.Split('\n');

        for (int l = 0; l < lines.Length; l++)
        {
          string[] wordPair = lines[l].Split(':');
          try
          {
            switch (wordPair[0].ToUpper())
            {
              case "NEWSTATE":
                if (wordPair[1].ToUpper() == "CLEARCOMM")
                {
                  _commands.Clear();
                  _currentNodeNum = 0;
                  _status = State.STANDBY;
                }
                else 
                  _status = StatusEnumFromString(wordPair[1].ToUpper());
                break;

              case "NEWCOMMAND":
                _commands.Add(wordPair[1].ToUpper());
                break;

              case "NEWSENDER":
                _sender = string.IsNullOrWhiteSpace(wordPair[1]) ? "NULL" : wordPair[1].Trim();
                break;

              case "NEWRECIPIENT":
                _recipient = string.IsNullOrWhiteSpace(wordPair[1]) ? "NULL" : wordPair[1].Trim();
                break;

              case "NEWMODE":
                _mode = wordPair[1];
                break;

              case "NEWRADIO":
                _radio = wordPair[1];
                break;

              case "CLEARROUTE": //TODO
                _currentNodeNum = 0;
                _route.Clear();
                NextCommand();
                break;

              default:
                break;
            }// end switch
          }//end for
          catch (Exception e)
          {
            ProgramInstance.Echo(e.ToString());
            ProgramInstance.Echo(e.StackTrace);
          }
        }
        WriteData();
      }
      #endregion

      #region "Initialization routine"
      T GetFirstWithName<T>(string name) where T : class, IMyTerminalBlock
      {
        List<T> units = new List<T>();
        ProgramInstance.GridTerminalSystem.GetBlocksOfType(units);
        foreach (T unit in units)
          if (unit.IsSameConstructAs(ProgramInstance.Me) && unit.CustomName.Contains(name))
            return unit;
        return null;
      }

      void InitializeAndSubscribeSuspensions()
      {
        List<IMyTerminalBlock> units = new List<IMyTerminalBlock>();

        ProgramInstance.GridTerminalSystem.GetBlocksOfType<IMyMotorSuspension>(units);
        foreach (IMyTerminalBlock unit in units)
          if (unit.IsSameConstructAs(ProgramInstance.Me) && unit.CubeGrid == ProgramInstance.Me.CubeGrid)
          {
            if (unit.CustomName.Contains("Right"))
            {
              SuspensionWrapper suspensionWrapper = 
                new SuspensionWrapper(unit as IMyMotorSuspension, ref _controller);
              ChangeTruckSuspensionParameters += suspensionWrapper.SuspensionParametersChanged;
              ChangeTruckSuspensionStrength += suspensionWrapper.SuspensionStrengthChanged;
              _suspensions.Add(suspensionWrapper);
            }
            else
            {
              SuspensionWrapper suspensionWrapper =
                  new SuspensionWrapper(unit as IMyMotorSuspension, ref _controller) { Side = -1 };
              ChangeTruckSuspensionParameters += suspensionWrapper.SuspensionParametersChanged;
              ChangeTruckSuspensionStrength += suspensionWrapper.SuspensionStrengthChanged;
              _suspensions.Add(suspensionWrapper);
            }
          }
        units.Clear();
      }

      internal MULE()
      {
        _evasive = 0;
        ProgramInstance.Me.CustomName = "PB CDS Drone";
        ProgramInstance.Me.CustomData = "IGC ID: " + ProgramInstance.IGC.Me;
        _connector = GetFirstWithName<IMyShipConnector>(_connectorName);
        _sensor = GetFirstWithName<IMySensorBlock>(_sensorName);
        _controller = GetFirstWithName<IMyRemoteControl>(_controllerName);

        InitializeAndSubscribeSuspensions();

        ReadRouteData();
        try { ReadData(); } catch { WriteData(); } 
        
      }
      #endregion

      #region "Movement and navigation"
      void GetForwardAxels()
      {
        _rollAxle = _controller.WorldMatrix.Forward;
        _pitchAxle = _controller.WorldMatrix.Left;
      }

      void GetBackwardAxels()
      {
        _rollAxle = _controller.WorldMatrix.Backward;
        _pitchAxle = _controller.WorldMatrix.Right;
      }

      void CalculatePlanarAngleAndDistanceTo(Vector3D target, out float angle, out double distance)
      {
        target = target - _gridCenterPosition;

        Vector3D RollProjection = Vector3D.ProjectOnVector(ref target, ref _rollAxle);
        Vector3D PitchProjection = Vector3D.ProjectOnVector(ref target, ref _pitchAxle);

        Vector3D planeProjection = RollProjection + PitchProjection;

        angle = (float)Math.Acos(Vector3D.Dot(planeProjection, _rollAxle) / (planeProjection.Length() * _rollAxle.Length()));
        angle = Vector3D.Dot(target, _pitchAxle) > 0 ? -angle : angle;

        double 
          rollPLength = RollProjection.Length(),
          pitchPLength = PitchProjection.Length();

        distance = Math.Sqrt(rollPLength * rollPLength + pitchPLength * pitchPLength);
      }

      void GetRelativeEntitySpeed(Vector3D target, out float speed) 
      {
        _rollAxle = _controller.WorldMatrix.Forward;
        speed = (float)Vector3D.ProjectOnVector(ref target, ref _rollAxle).Length();
        //direction = Vector3D.Dot(target, _pitchAxle) > 0 ? -1 : 1;
      }

      void WheelsControl(float angle, int direction, float speed)
      {
        if (float.IsNaN(angle))
          return;

        SuspensionParametersChangedEventArgs args = new SuspensionParametersChangedEventArgs();

        args.Speed = speed;

        float KmH = (float)_controller.GetShipSpeed() * 3.7f;

        _controller.HandBrake = KmH > args.Speed + 2.0f;// || _handBrake;

        float speedFactor = KmH / _maxForwardSpeed;
        args.Movement = -(1f - 0.7f * speedFactor) * direction;

        args.Friction = 50.0f - 25.0f * speedFactor;

        args.Steer = angle / MAX_STEER_ANGLE;
        if (float.IsNaN(args.Steer))
          args.Steer = 0;

        ChangeTruckSuspensionParameters?.Invoke(this, args);
      }

      /// <summary>
      /// Try to roll around of obstacle by right side
      /// </summary>
      
      void EvasiveManeuver()
      {
        MyDetectedEntityInfo entityInfo = _sensor.LastDetectedEntity;
        if (entityInfo.Type == MyDetectedEntityType.CharacterOther)
          return; //LOL! PUSH PEDAL TO A METAL!

        //TODO Set next Way point if distance to curren is less that 20m

        if (!_controller.HandBrake && _currentSpeed > 20.0f + 5.0f)
          _controller.HandBrake = true;
        else
          _controller.HandBrake = false;

        Vector3D evasivePoint = entityInfo.Position;

        Vector3D evasiveVector = _controller.WorldMatrix.Right * 55;

        evasivePoint += evasiveVector;

        GetForwardAxels();
        float angle;
        double distance;
        CalculatePlanarAngleAndDistanceTo(evasivePoint, out angle, out distance);
        //WheelsControl(-angle, 1);
      }

      void HoldDistance()
      {
        //TO DO

        // Check distance to object
        MyDetectedEntityInfo info = _sensor.LastDetectedEntity;
        double distance = (info.Position - _gridCenterPosition).Length();
        distance -= info.BoundingBox.HalfExtents.Length();
        distance -= _controller.CubeGrid.WorldAABB.HalfExtents.Length();

        int direction = 1;

        float
          angle = 0,
          speed = _route[_currentNodeNum - 1].Speed;

        // Is it far? - Drive closer.

        if (distance > 35)
        {
          GetForwardAxels();
          CalculatePlanarAngleAndDistanceTo(_route[_currentNodeNum].Position, out angle, out distance);

          if (distance < _route[_currentNodeNum].CheckDistance)
          {
            _currentNodeNum++;
            WriteData();
          }
        }

        // Is it safe? - Hold.
        else if (25 < distance && distance < 35d)
        {
          GetForwardAxels();
          GetRelativeEntitySpeed(info.Velocity, out speed);
          speed = speed > _route[_currentNodeNum - 1].Speed ? _route[_currentNodeNum - 1].Speed : speed;

          CalculatePlanarAngleAndDistanceTo(_route[_currentNodeNum].Position, out angle, out distance);
          if (distance < _route[_currentNodeNum].CheckDistance)
          {
            _currentNodeNum++;
            WriteData();
          }
        }

        // Is it to close? - Move backward to previous node.
        else
        {
          GetBackwardAxels();

          direction = -1;
          speed = 10;

          CalculatePlanarAngleAndDistanceTo(_route[_currentNodeNum - 1].Position, out angle, out distance);

          if (distance < _route[_currentNodeNum - 1].CheckDistance)
          {
            _currentNodeNum--;
            WriteData();
          }
        }

        _controller.HandBrake = _currentSpeed > speed + 5.0f;
        WheelsControl(angle * direction, direction, speed);
      }

      private void UdjustSuspensionStrength()
      {
        float gravityFactor = (float)_controller.GetNaturalGravity().Length() / 9.81f;
        float truckCurrentMass = _controller.CalculateShipMass().PhysicalMass * gravityFactor;
        float massToWheelsDifference = truckCurrentMass / _suspensions.Count;
        float _truckSuspensionStrength = (float)Math.Sqrt(massToWheelsDifference / _suspensionSoftnessFactor);

        SuspensionStrengthChangedEventArgs args = new SuspensionStrengthChangedEventArgs
        { Strength = _truckSuspensionStrength };
        ChangeTruckSuspensionStrength?.Invoke(this, args);

      }
      
      void MovementControl(Gear gear)
      {
        if (_currentNodeNum < _routeLength) // ??? used to prevent evasion while docking
          _sensor.FrontExtend = 2.0f + (float)_currentSpeed * 0.75f;
        else //we ready to connect
          _sensor.FrontExtend = (float)_distanceToTarget * 0.8f;

        float angle = 0;
        double distance;
        int
          direction = 1,
          nodeNum = _currentNodeNum;

        switch (gear)
        {
          case Gear.NEUTRAL:
            _controller.HandBrake = true;
            return;

          case Gear.FORWARD:
            if (!_sensor.IsActive)//way is clear
            {
              GetForwardAxels();
              direction = 1;
            }
            else
            {
              //EvasiveManeuver();
              HoldDistance();
              return;
            }
            break;

          case Gear.BACKWARD:
            GetBackwardAxels();
            direction = -1;
            break;
        }

        CalculatePlanarAngleAndDistanceTo(_route[_currentNodeNum].Position, out angle, out distance);

        WheelsControl(angle * direction, direction, _route[_currentNodeNum - 1].Speed);

        _controller.HandBrake = _currentSpeed > _route[_currentNodeNum - 1].Speed + 5.0f;

        if (distance < _route[_currentNodeNum].CheckDistance)
        {
          _connector.Enabled = false;
          _currentNodeNum++;

          if (_currentNodeNum == _routeLength && _commands[0] == "DOCKING")
          {
              _connector.Enabled = true;
              NextCommand();
          }
          if (_currentNodeNum == 3 && !string.IsNullOrWhiteSpace(_goodbayTarget))
          {
            string[] data = _goodbayTarget.Split(' ');
            ProgramInstance.IGC.SendBroadcastMessage(data[0],"GOODBAY " + data[1]);
            _goodbayTarget = "";
          }

          if (_currentNodeNum > _routeLength)
            NextCommand();

          WriteData();
        }
      }
      #endregion

      void MoveOrStop()
      {
        if (_status == State.MOVE || _status == State.DOCKING)
        {
          ProgramInstance.
          Runtime.UpdateFrequency = UpdateFrequency.Update1;
          UdjustSuspensionStrength();
          _currentNodeNum = 1;
          _controller.HandBrake = false;
        }
        else
        {
          MovementControl(Gear.NEUTRAL);
          ProgramInstance.
          Runtime.UpdateFrequency = UpdateFrequency.Update100;
        }
      }

      void NextCommand()
      {
        if (_commands.Count == 0)
          _status = State.STANDBY;
        else
        {
          _status = StatusEnumFromString(_commands[0]);
          _commands.RemoveAt(0);
        }
        MoveOrStop();
        WriteData();
      }

      void RadioInput() 
      {
        while (ProgramInstance.IGC.UnicastListener.HasPendingMessage) 
        {
          MyIGCMessage message = ProgramInstance.IGC.UnicastListener.AcceptMessage();
          if (message.Tag == "ROUTE")
            RouteData = message.Data.ToString();
          if (message.Tag == "COMMAND")
            NewInstructions(message.Data.ToString());
        }
      }

      internal void Main()
      {
        ProgramInstance.Echo("Status: " + _status);
        ProgramInstance.Echo("Route length: " + _routeLength);
        ProgramInstance.Echo("Current node: " + _currentNodeNum);

        _gridCenterPosition = _controller.CubeGrid.GetPosition();
        _currentSpeed = _controller.GetShipSpeed() * 3.7d;

        RadioInput();

        switch (_status)
        {
          case State.STANDBY:
            if (_commands.Count != 0)
              NextCommand();
            break;

          case State.MOVE:
            MovementControl(_route[_currentNodeNum].Direction);
            break;

          case State.CLEARCOMM:
            _commands.Clear();
            _currentNodeNum = 0;
            NextCommand();
            break;

          case State.CLEARROUTE: //TODO
            _currentNodeNum = 0;
            _route.Clear();
            NextCommand();
            break;

          case State.READROUTE:
            _currentNodeNum = 0;
            ReadRouteData();
            NextCommand();
            break;

          case State.UNDOCKING:
            if (_connector.Status == MyShipConnectorStatus.Connected)
            {
              _connector.Disconnect();
              _connector.Enabled = false;
            }
            else
              NextCommand();
            break;

          case State.DOCKING:
            GetForwardAxels();
            float angle; double distance;
            CalculatePlanarAngleAndDistanceTo(_route[_currentNodeNum].Position, out angle, out distance);
            WheelsControl(angle, 1, _route[_currentNodeNum - 1].Speed);


            if (_currentNodeNum == _routeLength)
            {
              if (_connector.Status == MyShipConnectorStatus.Connectable)
                _connector.Connect();
              if (_connector.Status == MyShipConnectorStatus.Connected)
                NextCommand();
            }
            else if (distance < _route[_currentNodeNum].CheckDistance)
            {
              _currentNodeNum++;
              _connector.Enabled = false;
              if (_currentNodeNum == _routeLength)
                _connector.Enabled = true;
              WriteData();
            }

            break;

          case State.RADIOCALL:
            ProgramInstance.IGC.SendBroadcastMessage(_radio, "DELIVERY " + _sender + " " + _mode + " " + _recipient);
            NextCommand();
            break;
        }
      }
    }

    void Main(string argument)
    {
      // initialize 
      unit.Main();
      if (!string.IsNullOrWhiteSpace(argument))
        unit.NewInstructions(argument);
      Echo("CIC: " + Runtime.CurrentInstructionCount.ToString());
    }

  }
}
