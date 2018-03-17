using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using AgX_Interface;
using System.Xml.Serialization;
using System.Drawing;
using System.IO;
//using UnityEngine.UI;

namespace Simulation_Core
{
    public class Scenario
    {
        public Robot robot;
        public Scene scene;
    }


    [XmlRoot("Robot", Namespace = "Assembly")]
    public class Robot
    {
        public List<Module> modules = new List<Module>();
        public List<Sensory_Module> sensorModules = new List<Sensory_Module>();
        public List<Joint> locks = new List<Joint>();

        public string leftFrameDir, rightFrameDir;

        internal Vector3 position;

        public void Add_Module(Module module)
        {
            modules.Add(module);
        }
        public void Add_Module(Module module, Joint lockjoint)
        {
            modules.Add(module);
            locks.Add(lockjoint);
        }

        public void Add_SensorModule(Sensory_Module module, Joint r_lockjoint)
        {//This just creates the locks, doesnt need more info.
            sensorModules.Add(module);
            locks.Add(r_lockjoint);
        }
        public void Add_SensorModule(Joint l_lockjoint, Sensory_Module module, Joint r_lockjoint)
        {
            sensorModules.Add(module);
            locks.Add(l_lockjoint);
            locks.Add(r_lockjoint);
        }

        public void Initialize()//Initializes lock joints.
        {
            //Creates joints between frames and inits frame objects:
            foreach (Module mod in modules)
            {
                mod.Initialize();
            }
            foreach (Sensory_Module mod in sensorModules)
            {
                mod.Initialize();
            }
            //if sensormodule on 0, attach to first module. 

            int lockNrCount = 0;//USE THIS

            //Sets locks between modules:
            for (int i = 0; i < modules.Count; i++)
            {

                if (sensorModules.Any(x => x.rightMod_Nr == i))//if this module is to the right of a sensor moduleif(sensorModules.Any(x => x.rightMod_Nr == modules[i].number)
                {
                    locks[lockNrCount].Create_SensorModuleLock(sensorModules.Find(x => x.rightMod_Nr == i), modules[i].frames[0]);
                    lockNrCount++;
                }

                if (sensorModules.Any(x => x.leftMod_Nr == i))//if this module is to the left of a sensor module:
                {
                    locks[lockNrCount].Create_SensorModuleLock(modules[i].frames[1], sensorModules.Find(x => x.leftMod_Nr == i));
                    lockNrCount++;
                }

                if (i + 1 < modules.Count && !sensorModules.Any(x => x.leftMod_Nr == i || x.rightMod_Nr == i))//If the iterator is not exceeding module count, and this module and has no right or left to any sensor
                {
                    locks[lockNrCount].Create_Lock(modules[i].frames[1], modules[i + 1].frames[0]);
                    lockNrCount++;
                }

            }
            //Set Pitch or Yaw
            foreach (Module module in modules)
                module.Axis = module.frames[0].rotation.z == 0 ? "Pitch" : "Yaw";

        }

        public void Update()
        {
            position = Vector3.zero;
            foreach (Module mod in modules)
            {
                mod.Update();
                position += mod.position;
            }
            position /= modules.Count;
            foreach (Sensory_Module mod in sensorModules)
            {
                mod.Update();
            }
        }
    }


    public class Module
    {
        public int mod_Nr;
        public Vector3 position;
        public string Axis;
        public Frame[] frames = new Frame[2];
        public Joint joint;

        public double z_leftEdge, z_rightEdge, top, bot;

        public void Create(Frame left, Joint joint, Frame right)//Creates are for Scecne designer, Initialize is for simulator
        {
            frames[0] = left; frames[1] = right;
            this.joint = joint;
        }

        public void Initialize()
        {
            foreach (Frame frame in frames)
                frame.Initialize();

            position = Vector3.Lerp(frames[0].position, frames[1].position, 0.5f);
            joint.Create_Hinge(frames[0], frames[1]);
        }

        public void Update()
        {
            foreach (Frame frame in frames)//Update all frames in the module
            {
                frame.Update();
            }

            //Update Joint: (There is always a joint)
            joint.Update();

            //Update module position
            position = Vector3.Lerp(frames[0].position, frames[1].position, 0.5f);
        }
    }

    public class Sensory_Module //Square
    {
        public Guid guid;
        public int leftMod_Nr, rightMod_Nr;

        public Vector3 position;
        public Vector3 rotation;
        public Vector3 size;
        public double mass;
        public string materialName;

        internal AgX_Primitive agxPrimitive;

        public void Initialize()
        {
            agxPrimitive = new AgX_Primitive(guid, "Box", position, rotation, size, mass, materialName);
        }
        public void Update()
        {
            position = agxPrimitive.Get_Position();
            rotation = agxPrimitive.Get_Rotation();
        }
    }


    public class Frame
    {
        public Guid guid;
        public string shape;

        public double scale;
        public Vector3 position;
        public Vector3 rotation;
        private Quaternion quatRotation;
        public double mass;
        public Boolean isStatic;
        public string materialName;

        public Vector3[] meshVertices; public Vector2[] meshUvs; public int[] meshTriangles;
        internal AgX_Frame agxFrame;

        public void Initialize() //Create frame object
        {
            ScaleMesh();
            agxFrame = new AgX_Frame(this.guid, shape, meshVertices, meshUvs, meshTriangles, scale, position, rotation, mass, isStatic, materialName);
        }

        private void ScaleMesh()
        {
            Vector3[] tmp_Vertices = meshVertices;
            for (int i = 0; i < tmp_Vertices.Length; i++)
            {
                tmp_Vertices[i].x *= scale;
                tmp_Vertices[i].y *= scale;
                tmp_Vertices[i].z *= scale;
            }
            meshVertices = tmp_Vertices;
        }

        public void Update()//Update variables
        {
            //scale = frame.Get_Size();
            position = agxFrame.Get_Position();
            rotation = agxFrame.Get_Rotation();
            quatRotation = agxFrame.Get_QuatRotation();
        }
        public Quaternion GetQuatRot()
        {
            return quatRotation;
        }
        public void setMesh(Vector3[] vertices, Vector2[] uvs, int[] triangles)
        {
            meshVertices = vertices;
            meshUvs = uvs;
            meshTriangles = triangles;
        }
    }


    public class Joint
    {
        public Guid guid, leftFrameGuid, rightFrameGuid;
        public string type;
        public Vector3 mid_Position;
        public double leftRangeLimit, rightRangeLimit;

        /*Alternative:*/
        public double Kp = 3;
        public double max_vel;

        internal AgX_Joint joint;
        internal Frame left, right;

        public void Create_Hinge(Frame left, Frame right)
        {
            this.left = left; this.right = right;
            type = "Hinge";
            joint = new AgX_Joint(guid);

            //Add joint between top and bottom frame
            joint.Create_Hinge("Hinge", left, right, leftRangeLimit, rightRangeLimit);
            joint.AddToSim();
        }
        public void Create_Lock(Frame left, Frame right)
        {
            this.left = left; this.right = right;
            type = "Lock";
            joint = new AgX_Joint(guid);
            joint.Create_Lock("Lock", left, right);
            joint.AddToSim();
        }

        //Creates an AgX joint that locks a frame and a sensor module together (agxObjects can not be referenced because references are not saved to this joint object). 
        public void Create_SensorModuleLock(Frame right, Sensory_Module s_mod)//sensor placed last, right frame of left module 
        {
            joint = new AgX_Joint(guid);
            joint.Create_Lock("Lock", right, s_mod);//FIX IN AGX INTERFACE
            joint.AddToSim();
        }
        public void Create_SensorModuleLock(Sensory_Module s_mod, Frame left)//sensor placed first, left frame of right module
        {
            joint = new AgX_Joint(guid);
            joint.Create_Lock("Lock", s_mod, left);//FIX IN AGX INTERFACE
            joint.AddToSim();
        }

        public void MOVE(double requested_angle)
        {
            double error = requested_angle - joint.Get_Angle();//Expand to pid if required later?

            if (Kp * error > max_vel)
                joint.Set_Speed(max_vel);
            else if (Kp * error < -max_vel)
                joint.Set_Speed(-max_vel);
            else
                joint.Set_Speed(Kp * error);
        }

        public void Reset_Angle()//Resets the joint movement
        {
            double error = 0 - joint.Get_Angle();

            if (Kp * error > 0.5f)
                joint.Set_Speed(0.5f);
            else if (Kp * error < -0.5f)
                joint.Set_Speed(-0.5f);
            else
                joint.Set_Speed(Kp * error);
        }

        public void Update()
        {
            //mid_Position = left.position - left.GetQuatRot() * Vector3.forward;// * left.scale.z;
        }
    }


    public class Scene
    {
        public Guid guid;
        public string height_Image;

        public List<Vector3> vertices = new List<Vector3>();
        public List<int> triangles = new List<int>();
        public Vector2[] uvs;

        public Vector3 position;
        public string materialName;
        public double height;
        //Water
        //Air

        Agx_Scene scene;
        public void Create()
        {
            TerrainFromImage();//Loads the heightmap

            scene = new Agx_Scene(guid, vertices, triangles, position, materialName, height);

        }

        private void TerrainFromImage()

        {
            // Modified from: https://answers.unity.com/questions/1033085/heightmap-to-mesh.html

            //Unity:
            //Texture2D heightMap = new Texture2D(250, 250);
            //heightMap.LoadImage(Convert.FromBase64String(height_Image));

            //C#,Visual Studio
            Bitmap heightMap = new Bitmap(250, 250);//What will this be..?
            using (var ms = new MemoryStream(Convert.FromBase64String(height_Image)))//C#/Visual Studio
            {
                heightMap = new Bitmap(ms);
            }

            //Bottom left section of the map, other sections are similar
            for (int i = 0; i < 250; i++)
            {
                for (int j = 0; j < 250; j++) //OUTER PIXELS MUST BE 0, fix
                {
                    //Add each new vertex in the plane
                    vertices.Add(new Vector3(i, heightMap.GetPixel(i, j).R * height, j));//can either take red, blue or green, because all are the same in grayscale.
                    //Skip if a new square on the plane hasn't been formed
                    if (i == 0 || j == 0) continue;
                    //Adds the index of the three vertices in order to make up each of the two tris
                    triangles.Add(250 * i + j); //Top right
                    triangles.Add(250 * i + j - 1); //Bottom right
                    triangles.Add(250 * (i - 1) + j - 1); //Bottom left - First triangle
                    triangles.Add(250 * (i - 1) + j - 1); //Bottom left 
                    triangles.Add(250 * (i - 1) + j); //Top left
                    triangles.Add(250 * i + j); //Top right - Second triangle
                }
            }

            uvs = new Vector2[vertices.Count];
            for (var i = 0; i < uvs.Length; i++) //Give UV coords X,Z world coords
                uvs[i] = new Vector2(vertices[i].x, vertices[i].z);

        }

        void Add()
        {
            //scenario.Add_Rb();
        }
    }


    public class ForceSensor
    {
        public Guid guid;
        public double value;
        public Vector3 position;
        public Vector3 scale;

        internal AgX_Sensor agxSensor;
        internal AgX_Joint lockJoint;

        public void Initialize()
        {
            agxSensor = new AgX_Sensor(guid, "Plastic", position, scale, 1.0f);
            lockJoint = new AgX_Joint(guid);//Ikke festet

        }

        public void Lock(Frame frame)
        {
            lockJoint.SensorLock(frame, this);
            lockJoint.AddToSim();
        }

        public void Update()
        {
            position = agxSensor.GetPosition();
            //rotation
        }
    }

    class DistanceSensor
    {
        void Initialize()
        {

        }
    }

    /*-------------------------------------------------Utility Functions:-------------------------------------------------*/
    /*------------------------------------------------------Vector3-------------------------------------------------------*/

    public struct Vector3
    {
        public double x, y, z;

        public static Vector3 forward => new Vector3(0, 0, 1);
        public static Vector3 zero => new Vector3(0f, 0f, 0f);

        public Vector3(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static double Length(Vector3 a)
        {
            return Math.Sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        }

        public static Vector3 Normalize(Vector3 a)
        {
            double length = Length(a);
            if (length != 0)
            {
                a.x = a.x / length;
                a.y = a.y / length;
                a.z = a.z / length;
            }
            return a;
        }

        public static Vector3 Lerp(Vector3 a, Vector3 b, double amount)
        {
            return amount * Vector3.Normalize(b - a) + a;
        }

        //Operators:
        public static Vector3 operator +(Vector3 a, Vector3 b)
        {
            a.x += b.x;
            a.y += b.y;
            a.z += b.z;
            return a;
        }
        public static Vector3 operator -(Vector3 a, Vector3 b)
        {
            a.x -= b.x;
            a.y -= b.y;
            a.z -= b.z;
            return a;
        }
        public static Vector3 operator *(Vector3 a, double scale)
        {
            a.x *= scale;
            a.y *= scale;
            a.z *= scale;
            return a;
        }
        public static Vector3 operator *(double scale, Vector3 a)
        {
            a.x *= scale;
            a.y *= scale;
            a.z *= scale;
            return a;
        }
        public static Vector3 operator /(Vector3 a, double scale)
        {
            a.x /= scale;
            a.y /= scale;
            a.z /= scale;
            return a;
        }


    }

    public struct Vector2
    {
        public double x, y;

        public Vector2(double x, double y)
        {
            this.x = x;
            this.y = y;
        }
    }

    public struct Quaternion
    {
        public double x, y, z, w;

        static Quaternion identity = new Quaternion(0, 0, 0, 1);

        public Quaternion(double x, double y, double z, double w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }



        public Vector3 eulerAngles()
        {
            Vector3 vector;

            double unit = x * x + y * y + z * z + w * w;
            double test = x * y + z * w;

            if (test > 0.4999f * unit)                              // 0.4999f OR 0.5f - EPSILON
            {
                // Singularity at north pole
                vector.y = 2f * (float)Math.Atan2(x, w);  // Yaw
                vector.x = Math.PI * 0.5f;                         // Pitch
                vector.z = 0f;                                // Roll
                return vector;
            }
            else if (test < -0.4999f * unit)                        // -0.4999f OR -0.5f + EPSILON
            {
                // Singularity at south pole
                vector.y = -2f * (float)Math.Atan2(x, w); // Yaw
                vector.x = -Math.PI * 0.5f;                        // Pitch
                vector.z = 0f;                                // Roll
                return vector;
            }
            else
            {

                vector.y = (float)Math.Atan2(2f * x * w + 2f * y * z, 1 - 2f * (z * z + w * w));     // Yaw 
                vector.x = (float)Math.Asin(2f * (x * z - w * y));                             // Pitch 
                vector.z = (float)Math.Atan2(2f * x * y + 2f * z * w, 1 - 2f * (y * y + z * z));      // Roll 

                return vector;
            }
        }
    }
}
