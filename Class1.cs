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
        public Program()
        { Runtime.UpdateFrequency = UpdateFrequency.Update10; }
        
        public enum Status { STANDBY , MOVE , BASEMOVE , CLEARCOMM , CLEARROUTE , READROUTE , DOCKING , UNDOCKING, DELIVER };
        public enum Gear { NEUTRAL , FORWARD , BACKWARD};

        public class MULE
        {
            private Program program;

            private IMyShipConnector connector;
            private string connectorName = "Connector Forward";

            private IMyTextPanel statusPanel;
            private string statusPanelName = "Status";

            private IMyTextPanel routePanel;
            private string routePanelName = "Route";

            private IMySensorBlock sensor;
            private string sensorName = "Sensor";

            private IMyRemoteControl controller;
            private string remoteControlName = "RemoteControl";

            private List<IMyMotorSuspension> wheels;
            private Vector3D myPos;

            private Status status;
            private string previousInput;

            private string mySender;
            private string myRecipient;

            private List<string> commands;

            private int evasive;
            private double currentSpeed;
            private int routeNodesCount;
            private int currentNodeNum;
            private double distanceToNode;

            private List<string> routeNames;
            private List<Vector3D> routeNodes;
            private List<double> routeSpeedLimits;
            private List<double> routeApproachRadiuses;
            private List<Gear> routeDirection;

            private IMyTerminalBlock GetFirstWithName(string name)
            {
                List<IMyTerminalBlock> units = new List<IMyTerminalBlock>();
                program.GridTerminalSystem.SearchBlocksOfName(name, units);
                foreach (IMyTerminalBlock unit in units)
                    if (unit.IsSameConstructAs(program.Me))
                        return unit;
                return null;
            }

            private List<IMyMotorSuspension> InitMotors()
            {
                List<IMyMotorSuspension> motors = new List<IMyMotorSuspension>();
                program.GridTerminalSystem.GetBlocksOfType(motors);
                if (motors.Count == 0)
                    program.Echo("\n- Grid have no wheels");

                List<IMyMotorSuspension> returnedMotors = new List<IMyMotorSuspension>();

                foreach (IMyMotorSuspension motor in motors)
                    if (motor.IsSameConstructAs(program.Me))
                        returnedMotors.Add(motor);
                motors.Clear();
                return returnedMotors;
            }

            private void ReadRouteData()
            {
                if (routePanel == null)
                    program.Echo("\n- Shit happend RoutePanel == null."); //to do
                var text = routePanel.GetText();
                if (text == null)
                    program.Echo("\n- Route data is empty."); //to do

                routeNames = new List<string>();
                routeNodes = new List<Vector3D>();
                routeSpeedLimits = new List<double>();
                routeApproachRadiuses = new List<double>();
                routeDirection = new List<Gear>();

                string[] lines = text.Split('\n');
                for (int l = 0; l < lines.Length; l++)
                {
                    string[] words = lines[l].Split(':');
                    //0  1   2    3        4         5
                    //speed:radius:name:X:Y:Z:
                    //20:3.5:direction:Node:15934.19:-15396.17:16499.03:
                    //will be changed 
                    //speed:radius:direction:possible evasive:name:X:Y:Z:
                    try
                    {
                        routeSpeedLimits.Add(Convert.ToDouble(words[0]));
                        routeApproachRadiuses.Add(Convert.ToDouble(words[1]));
                        routeDirection.Add(GearEnumFormInt(Convert.ToInt32(words[2])));
                        routeNames.Add(words[3]);
                        routeNodes.Add( new Vector3D(
                            Convert.ToDouble(words[4]),
                            Convert.ToDouble(words[5]),
                            Convert.ToDouble(words[6])
                            ));
                    }
                    catch (Exception e)
                    {
                        program.Echo(e.ToString());
                    }
                }
                routeNodesCount = routeNodes.Count - 1;
            }

            private Status StatusEnumFromString(string strStatus)
            {
                switch (strStatus.ToUpper()) {
                    case "BASEMOVE":
                        return Status.BASEMOVE;
                    case "CLEARCOMM":
                        return Status.CLEARCOMM;
                    case "CLEARROUTE":
                        return Status.CLEARROUTE;
                    case "DOCKING":
                        return Status.DOCKING;
                    case "MOVE":
                        return Status.MOVE;
                    case "READROUTE":
                        return Status.READROUTE;
                    case "UNDOCKING":
                        return Status.UNDOCKING;
                    case "DELIVER":
                        return Status.DELIVER;
                }
                return Status.STANDBY;
            }

            private Gear GearEnumFormInt(int input)
            {
                switch (input)
                {
                    case 1: return Gear.FORWARD;
                    case 2: return Gear.BACKWARD;
                }
                return Gear.NEUTRAL;
            }

            private void ReadData()
            {
                mySender = "NULL";
                myRecipient = "NULL";
                status = Status.STANDBY;
                currentNodeNum = 0;
                commands = new List<string>();

                var text = statusPanel.GetText();
                previousInput = text;

                string[] lines = text.Split('\n');

                for (int l = 0; l < lines.Length; l++)
                {
                    string[] wordPair = lines[l].Split(':');
                    try
                    {
                        switch (wordPair[0].ToUpper())
                        {
                            case "STATE":
                                status = StatusEnumFromString(wordPair[1]);
                                break;

                            case "CURRPATH":
                                currentNodeNum = int.Parse(wordPair[1]);
                                break;

                            case "SENDER":
                                mySender = wordPair[1];
                                break;

                            case "RECIPIENT":
                                myRecipient = wordPair[1];
                                break;

                            case "NEXTCOMM":
                                commands.Add(wordPair[1].ToUpper());
                                break;

                            case "NEWSTATE":
                                status = StatusEnumFromString(wordPair[1]);
                                break;

                            case "NEWCOMMAND":
                                commands.Add(wordPair[1].ToUpper());
                                break;

                            case "NEWSENDER":
                                mySender = wordPair[1];
                                break;

                            case "NEWRECIPIENT":
                                myRecipient = wordPair[1];
                                break;

                            default:
                                break;
                        }// end switch
                    }
                    catch
                    (Exception e)
                    {
                        program.Echo("At:" + l + "\n" + e.ToString());
                    }
                }//end for
            }

            private void WriteData()
            {
                if (statusPanel != null)
                {
                    string data = "Sender:" + mySender + ":\n";
                    data = data + "Recipient:" + myRecipient + ":\n";
                    data = data + "State:" + status + ":\n";
                    data = data + "CurrPath:" + currentNodeNum + ":\n";

                    if (commands != null)
                        foreach (string command in commands)
                            data = data + "NextComm:" + command + ":\n";

                    data = data + "ConnStat:" + connector.Status.ToString() + ":\n";

                    statusPanel.WriteText(data, false);
                }
            }

            private void ReadInput(string inputCommand)
            {
                // Recive suddenly command
                if (inputCommand != "")
                    status = StatusEnumFromString(inputCommand);
                else
                // Just read that damned panel
                if (statusPanel != null)
                {

                    var text = statusPanel.GetText();
                    previousInput = text;

                    string[] lines = text.Split('\n');

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
                                        commands.Clear();
                                        currentNodeNum = 0;
                                        status = Status.STANDBY;
                                    }
                                    else status = StatusEnumFromString(wordPair[1].ToUpper());
                                    break;

                                case "NEWCOMMAND":
                                    commands.Add(wordPair[1].ToUpper());
                                    break;

                                case "NEWSENDER":
                                    mySender = wordPair[1];
                                    break;

                                case "NEWRECIPIENT":
                                    myRecipient = wordPair[1];
                                    break;

                                default:
                                    break;
                            }// end switch
                        }//end for
                        catch (Exception e)
                        {
                            program.Echo(e.ToString());
                        }
                    }
                    WriteData();
                }
            }

            public MULE(Program nProgram)
            {
                program = nProgram;

                evasive = 0;

                connector = GetFirstWithName(connectorName) as IMyShipConnector;
                statusPanel = GetFirstWithName(statusPanelName) as IMyTextPanel;
                routePanel = GetFirstWithName(routePanelName) as IMyTextPanel;
                sensor = GetFirstWithName(sensorName) as IMySensorBlock;

                wheels = InitMotors();
                if (wheels.Count == 0)
                    program.Echo("\n- Vehicle have no wheels.");

                if (routePanel != null)
                    ReadRouteData();
                previousInput = "";
                if (statusPanel != null)
                    ReadData();

                controller = GetFirstWithName(remoteControlName) as IMyRemoteControl;
            }

            private Vector3D MatrixXVector(MatrixD M, Vector3D V)
            {
                return new Vector3D(
                    M.M11 * V.X + M.M21 * V.Y + M.M31 * V.Z,
                    M.M12 * V.X + M.M22 * V.Y + M.M32 * V.Z,
                    M.M13 * V.X + M.M23 * V.Y + M.M33 * V.Z
                    );
            }

            /// <summary>
            /// Missile rotation control
            /// </summary>
            private double RotationControl(Vector3D target, double direction)
            {
                Vector3D forward = new Vector3D(
                  -(controller as IMyEntity).WorldMatrix.M31 * direction,
                  -(controller as IMyEntity).WorldMatrix.M32 * direction,
                  -(controller as IMyEntity).WorldMatrix.M33 * direction);

                Vector3D wing = new Vector3D(
                  -(controller as IMyEntity).WorldMatrix.M11 * direction,
                  -(controller as IMyEntity).WorldMatrix.M12 * direction,
                  -(controller as IMyEntity).WorldMatrix.M13 * direction);

                target = target - controller.GetPosition();
                target /= target.Length();

                // Rotation around X Axis ######################

                MatrixD M;
                double cos = 0.0d;
                double sin = 0.0d;
                double r = 0.0d;

                r = Math.Sqrt(forward.Y * forward.Y + forward.Z * forward.Z);

                cos = forward.Y / r;
                sin = forward.Z / r;

                M = new MatrixD(
                    1.0d, 0.0d, 0.0d,
                    0.0d, cos, -sin,
                    0.0d, sin, cos
                );

                forward = MatrixXVector(M, forward);
                target = MatrixXVector(M, target);
                wing = MatrixXVector(M, wing);

                // Rotation around Z Axis ######################

                r = Math.Sqrt(forward.X * forward.X + forward.Y * forward.Y);

                cos = forward.X / r;
                sin = forward.Y / r;

                M = new MatrixD(
                    cos, -sin, 0.0d,
                    sin, cos, 0.0d,
                    0.0d, 0.0d, 1.0d
                );

                target = MatrixXVector(M, target);
                wing = MatrixXVector(M, wing);

                // Rotation around X Axis ######################

                r = Math.Sqrt(wing.Y * wing.Y + wing.Z * wing.Z);

                cos = wing.Y / r;
                sin = wing.Z / r;

                M = new MatrixD(
                    1.0d, 0.0d, 0.0d,
                    0.0d, cos, -sin,
                    0.0d, sin, cos
                );

                target = MatrixXVector(M, target);

                // ########################################

                double Azimuth = Math.Acos(target.X / Math.Sqrt(target.X * target.X + target.Y * target.Y))
                  / Math.PI * 180.0d;
                if (target.Y > 0.0d) Azimuth = -Azimuth;

                program.Echo("Azimuth:" + Math.Round(Azimuth, 2));

                return Azimuth * direction;
            }

            private void WheelsControl(double speedLimit, double direction, double angle)
            {
                float friction = 25.0f;

                if (currentSpeed > 40.0f)
                    friction = 15.0f + 40.0f * (1.0f - 40.0f / (float)currentSpeed);
                else friction = 65.0f;

                foreach (IMyMotorSuspension motor in wheels)
                {
                    if (motor != null)
                    {
                        if (!motor.Enabled)
                            motor.Enabled = true;
                        if (motor.CustomName.Contains("Left"))
                            motor.SetValueFloat("Propulsion override", 1.0f * (float)direction);
                        if (motor.CustomName.Contains("Right"))
                            motor.SetValueFloat("Propulsion override", -1.0f * (float)direction);
                        if (motor.CustomName.Contains("Forward"))
                        {
                            motor.SetValueFloat("Steer override", Math.Sign(angle));
                            motor.SetValueFloat("MaxSteerAngle", (float)Math.Abs(angle));
                        }
                        motor.SetValueFloat("Friction", friction);
                        motor.SetValue<Single>("Speed Limit", (float)speedLimit - 1.0f);
                    }
                } // end foreach
            }

            /// <summary>
            /// Try to roll around of obstacle by right side
            /// </summary>
            private void EvasiveManeuver()
            {
                //TODO Set next Way point if distance to curren is less that 20m
                if (!controller.HandBrake && currentSpeed > 40.0f + 5.0f)
                    controller.HandBrake = true;
                else
                    controller.HandBrake = false;

                MyDetectedEntityInfo entityInfo = sensor.LastDetectedEntity;
                //if (entityInfo.Type == MyDetectedEntityType.CharacterOther) LOL! PUT PEDAL TO METAL!
                Vector3D evasivePoint = new Vector3D(
                  entityInfo.Position.X,
                  entityInfo.Position.Y,
                  entityInfo.Position.Z);

                IMyEntity myEntity = controller as IMyEntity;

                Vector3D evasiveVector = new Vector3D(
                  myEntity.WorldMatrix.M11 * 55.5d,
                  myEntity.WorldMatrix.M12 * 55.5d,
                  myEntity.WorldMatrix.M13 * 55.5d);

                evasivePoint += evasiveVector;

                double angle = RotationControl(evasivePoint, -1);
                WheelsControl(35.0d, 0.7d, angle);
            }

            private void HoldDistance()
            {
                //TO DO

                // Check distance to object
                double distance = (sensor.LastDetectedEntity.Position - myPos).Length();

                // Is it far? - Drive closer.
                if (distance > 17.5d && !controller.HandBrake)
                {
                    double angle = RotationControl(routeNodes[currentNodeNum], 1.0d);
                    WheelsControl(routeSpeedLimits[currentNodeNum - 1] / 3, 0.7d, angle);
                    controller.HandBrake = true;
                }

                // Is it safe? - Hold.
                if (12.5d < distance && distance < 17.5d)
                    controller.HandBrake = true;

                // Is it to close? - Move backward to previous node.
                if (distance < 12.5d)
                {
                    controller.HandBrake = false;
                    double angle = RotationControl(routeNodes[currentNodeNum - 1], -1.0d);
                    WheelsControl(routeSpeedLimits[currentNodeNum - 1] / 3, -0.7d, angle);
                }
            }

            private void FollowTheBackway()
            {
            }

            private void PreventCollision()
            {
                // if (there is enougth space to move around ) 
                //  EvasiveManeuver();
                // if angle absolute differense between me and object is not highter than 35*
                //  FollowTheBackway()
                // if there is no space around
                //  HoldDistance()
            }

            private void MovementControl(Gear gear)
            {

                if (currentNodeNum < routeNodes.Count - 1) // we are on run
                    sensor.FrontExtend = 2.0f + (float)currentSpeed;
                else //we ready to connect
                    sensor.FrontExtend = float.Parse(distanceToNode.ToString()) * 0.8f;

                if (gear == Gear.NEUTRAL)
                {
                    if (!controller.HandBrake)
                        controller.HandBrake = true;
                    foreach (IMyMotorSuspension motor in wheels)
                    {
                        if (!motor.Enabled)
                            motor.GetActionWithName("OnOff_On").Apply(motor);
                        motor.SetValueFloat("Propulsion override", 0.0f);
                    }
                }

                if (gear == Gear.FORWARD)
                {
                    // way is clear
                    if (!sensor.IsActive)
                    {
                        if (controller.HandBrake && currentSpeed < routeSpeedLimits[currentNodeNum - 1] + 5.0f)
                            controller.HandBrake = false;
                        // to fast
                        if (!controller.HandBrake && currentSpeed > routeSpeedLimits[currentNodeNum - 1] + 5.0f)
                            controller.HandBrake = true;
                        double angle = RotationControl(routeNodes[currentNodeNum], 1.0d);
                        WheelsControl(routeSpeedLimits[currentNodeNum - 1], 0.7d, angle);
                    }

                    else
                    {
                        // Collision imminent on the road
                        if (currentNodeNum <= routeNodes.Count - 3 && status != Status.BASEMOVE)
                            EvasiveManeuver();
                        // Collision imminent in base site 

                        // It seems to be a good way to prevent collision in narrow areas.
                        // It can be removed to more complex function of preventing collisions.
                        if (currentNodeNum > routeNodes.Count - 3)
                            HoldDistance();

                    } //end if (!sensor.IsActive) else 
                }

                if (gear == Gear.BACKWARD)
                {
                    if (controller.HandBrake)
                        controller.HandBrake = false;
                    double angle = (float)RotationControl(routeNodes[currentNodeNum], -1.0d);
                    WheelsControl(routeSpeedLimits[currentNodeNum - 1], -0.7d, angle);
                }

            }

            private void NextCommand()
            {
                if (commands.Count == 0)
                    status = Status.STANDBY;
                else
                {
                    status = StatusEnumFromString(commands[0]);
                    if (status == Status.BASEMOVE || status == Status.MOVE)
                        currentNodeNum = 1;
                    //pass zero route node since it not being used in most cases
                    commands.RemoveAt(0);
                }
            }

            private void RouteControl()
            {
                distanceToNode = (routeNodes[currentNodeNum] - myPos).Length();
                currentSpeed = controller.GetShipSpeed() * 3.7d;

                if ((connector.Status == MyShipConnectorStatus.Connectable
                || distanceToNode < routeApproachRadiuses[routeNodesCount]) && currentNodeNum == routeNodesCount)
                    NextCommand();

                if (connector.Enabled & currentNodeNum != routeNodesCount)
                    connector.GetActionWithName("OnOff_Off").Apply(connector);
                if (!connector.Enabled & currentNodeNum == routeNodesCount)
                    connector.GetActionWithName("OnOff_On").Apply(connector);

                if (distanceToNode < routeApproachRadiuses[currentNodeNum])
                    if (currentNodeNum <= routeNodesCount)
                        currentNodeNum++;

                if (status == Status.MOVE || status == Status.BASEMOVE)
                    MovementControl(routeDirection[currentNodeNum]);
            }

            private void State()
            {
                if (status == Status.STANDBY & commands.Count != 0)
                    NextCommand();

                if (status != Status.MOVE & status != Status.BASEMOVE)
                    MovementControl(Gear.NEUTRAL);

                if (status == Status.MOVE || status == Status.BASEMOVE)
                    RouteControl();

                if (status == Status.CLEARCOMM)
                {
                    commands.Clear();
                    currentNodeNum = 0;
                    NextCommand();
                }

                if (status == Status.CLEARROUTE)
                {
                    currentNodeNum = 0;
                    NextCommand();
                }

                if (status == Status.READROUTE)
                {
                    currentNodeNum = 0;
                    ReadRouteData();
                    NextCommand();
                }

                if (status == Status.UNDOCKING)
                {
                    if (connector.Status == MyShipConnectorStatus.Connected)
                        connector.Disconnect();
                    if (connector.Status == MyShipConnectorStatus.Unconnected || connector.Status == MyShipConnectorStatus.Connectable)
                        NextCommand(); // myStatus = "STANDBY";
                }

                if (status == Status.DOCKING)
                {
                    if (connector.Status == MyShipConnectorStatus.Connectable)
                        connector.Connect();
                    if (connector.Status == MyShipConnectorStatus.Connected)
                        NextCommand();
                }
            }

            public void Procced()
            {
                myPos = controller.GetPosition();
                //do stuff
                ReadInput("");
                State();
                //save status
                WriteData();
            }
        }

        MULE unit;
        bool initialize = false;
        void Main(string argument)
        {
            // initialize 
            if (!initialize)
            {
                unit = new MULE(this);
                initialize = true;
            }
            unit.Procced();
            Echo(Runtime.CurrentInstructionCount.ToString());
        }

    }
}
