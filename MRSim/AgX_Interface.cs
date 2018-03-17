/*Algoryx Interface class (AgX_Interface)
 * Torstein Sundnes Lenerand
 * NTNU Ålesund
 */

///This class contains the AgX object. 

using System;
using System.Collections.Generic;
using System.Linq;
using Simulation_Core;

namespace AgX_Interface
{


    public class AgX_Joint
    {
        private Guid guid;
        private string type;

        private agx.Constraint Joint;

        private agx.HingeFrame hinge_Frame = new agx.HingeFrame();//Might not need to be global

        public AgX_Joint(Guid guid)
        {
            this.guid = guid;
        }

        public void Create_Hinge(string type, Frame left, Frame right, double leftLimit, double rightLimit)
        {
            this.type = type;

            //Hinge is locked between the two objects.
            hinge_Frame.setCenter((left.agxFrame.GetAgxObject().getPosition() + right.agxFrame.GetAgxObject().getPosition()).Divide(2));
            if (left.rotation.z == 0)
                hinge_Frame.setAxis(new agx.Vec3(1, 0, 0)); //axis along the x direction
            else
                hinge_Frame.setAxis(new agx.Vec3(0, 1, 0)); //axis along the x direction
            Joint = new agx.Hinge(hinge_Frame, left.agxFrame.GetAgxObject(), right.agxFrame.GetAgxObject());
            //Joint.asHinge().getLock1D().setEnable(true);
            Joint.asHinge().getMotor1D().setEnable(true);
            Joint.asHinge().getRange1D().setEnable(true);
            //Might want to have this as a modifyable parameter:
            Joint.asHinge().getRange1D().setRange(leftLimit, rightLimit/*-Math.PI / 2, Math.PI / 2*/);

            //Joint.asHinge().getMotor1D().setSpeed(0.2f);
        }
        public void Create_Lock(string type, Frame left, Frame right)
        {
            //connects right frame of left robot (LEFT) to left frame of right robot (RIGHT)
            Joint = new agx.LockJoint(left.agxFrame.GetAgxObject(), right.agxFrame.GetAgxObject(), (left.agxFrame.GetAgxObject().getPosition() + right.agxFrame.GetAgxObject().getPosition()).Divide(2));
        }

        //Sensory module lock:
        public void Create_Lock(string type, Frame right, Sensory_Module s_mod)
        {
            //Creates a joint with a specified middle position for the lockframe.
            //THIS IS NOT THE MIDDLE OF THE LOCK FRAME (frames are longer than sensors)
            Joint = new agx.LockJoint(right.agxFrame.GetAgxObject(), s_mod.agxPrimitive.GetAgxObject(), (right.agxFrame.GetAgxObject().getPosition() + s_mod.agxPrimitive.GetAgxObject().getPosition()).Divide(2));
        }
        public void Create_Lock(string type, Sensory_Module s_mod, Frame left)
        {
            //THIS IS NOT THE MIDDLE OF THE LOCK FRAME (frames are longer than sensors)
            Joint = new agx.LockJoint(s_mod.agxPrimitive.GetAgxObject(), left.agxFrame.GetAgxObject(), (left.agxFrame.GetAgxObject().getPosition() + s_mod.agxPrimitive.GetAgxObject().getPosition()).Divide(2));
        }

        public void SensorLock(Frame frame, ForceSensor sensor)
        {
            Joint = new agx.LockJoint(frame.agxFrame.GetAgxObject(), sensor.agxSensor.GetAgxObject(), Operations.ToAgxVec3(sensor.position));
        }

        public double Get_Angle()
        {
            return (double)Joint.asHinge().getAngle();
        }
        public void Set_Speed(double vel)
        {
            Joint.asHinge().getMotor1D().setSpeed(vel);
        }

        public void AddToSim()
        {
            Agx_Simulation.sim_Instance.add(Joint);
        }

    }

    public class AgX_Sensor
    {
        private Guid guid;
        private Vector3 scale;
        private agx.RigidBody agx_Object;

        public AgX_Sensor(Guid guid, string materialName, Vector3 pos, Vector3 scale, double mass)
        {
            this.guid = guid;
            this.scale = scale;

            var dynamicRBGeometry = new agxCollide.Geometry();///AgX

            dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(scale)));

            dynamicRBGeometry.setMaterial(new agx.Material(materialName));

            agx_Object = new agx.RigidBody();

            agx_Object.add(dynamicRBGeometry);

            agx_Object.setLocalPosition(Operations.ToAgxVec3(pos));

            agx_Object.getMassProperties().setMass(mass);

            Agx_Simulation.sim_Instance.add(agx_Object);
        }

        public agx.RigidBody GetAgxObject()
        {
            return agx_Object;
        }
        public Vector3 GetPosition()
        {
            return Operations.FromAgxVec3(agx_Object.getLocalPosition());
        }
    }

    public class AgX_Primitive
    {
        private Guid guid;
        private string shape;
        private Vector3 size;
        private string materialName;

        private agx.RigidBody agx_Object;

        public AgX_Primitive(Guid guid, string shape, Vector3 pos, Vector3 rot, Vector3 size, double mass, string materialName)
        {
            this.guid = guid;
            this.shape = shape;
            this.size = size;
            this.materialName = materialName;

            var dynamicRBGeometry = new agxCollide.Geometry();

            switch (shape)
            {
                case "Box": dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(this.size))); break;
                case "Sphere": dynamicRBGeometry.add(new agxCollide.Sphere((this.size.x + this.size.y + this.size.z) / 3)); break;
            }

            dynamicRBGeometry.setMaterial(new agx.Material(materialName));

            agx_Object = new agx.RigidBody();
            agx_Object.add(dynamicRBGeometry);
            agx_Object.setLocalPosition(Operations.ToAgxVec3(pos));///AgX

            agx_Object.setLocalRotation(new agx.EulerAngles(Operations.ToAgxVec3(rot)));///AgX

            agx_Object.getMassProperties().setMass(mass);

            AddToSim();
        }

        public Vector3 Get_Position()
        {
            return Operations.FromAgxVec3(agx_Object.getLocalPosition());
        }
        public Vector3 Get_Rotation()
        {
            return Operations.FromAgxQuat(agx_Object.getLocalRotation()).eulerAngles();
        }
        public void AddToSim()
        {
            Agx_Simulation.sim_Instance.add(agx_Object);
        }
        public agx.RigidBody GetAgxObject()
        {
            return agx_Object;
        }
    }


    public class AgX_Frame
    {
        private Guid guid;
        public string shape;
        private double size;
        private string materialName;

        private agx.RigidBody agx_Object;

        public AgX_Frame(Guid guid, string shape, Vector3[] vertices, Vector2[] uvs, int[] triangles, double size, Vector3 pos, Vector3 rot, double mass, bool isStatic, string materialName)
        {
            this.guid = guid;

            this.shape = shape;
            this.size = size;
            this.materialName = materialName;

            //scale by 2, to fit unity.
            /* Vector3[] tmp_verts = vertices;
             for (int i = 0; i < tmp_verts.Length; i++)
             {
                 tmp_verts[i].x *= 2;
                 tmp_verts[i].y *= 2;
                 tmp_verts[i].z *= 2;
             }*/

            var tri = new agxCollide.Trimesh(Operations.ToAgxVec3Vector(vertices), Operations.ToAgxIntVector(triangles), "stdFrame");
            ///Creates a geometry
            var dynamicRBGeometry = new agxCollide.Geometry();
            dynamicRBGeometry.add(tri);

            dynamicRBGeometry.setMaterial(new agx.Material(materialName));

            ///Creates the selected shape
            /*switch (shape)
            {
                case "Box": dynamicRBGeometry.add(new agxCollide.Box(Operations.ToAgxVec3(size))); break;
                case "Sphere": dynamicRBGeometry.add(new agxCollide.Sphere((size.x + size.y + size.z) / 3)); break;
            }*/

            agx_Object = new agx.RigidBody();
            ///Adds selected geometry to the rigidbody
            agx_Object.add(dynamicRBGeometry);

            agx_Object.setLocalPosition(Operations.ToAgxVec3(pos));///AgX

            agx_Object.setLocalRotation(new agx.EulerAngles(Operations.ToAgxVec3(rot)));///AgX

            agx_Object.getMassProperties().setMass(mass);



            if (isStatic)
                agx_Object.setMotionControl(agx.RigidBody.MotionControl.STATIC);

            AddToSim();
        }

        /*--------------------------------------------------Object handling---------------------------------------------------*/
        /**-----------------------------------------------Return Algoryx object-----------------------------------------------*/
        public agx.RigidBody GetAgxObject()
        {
            return agx_Object;
        }

        public Guid Get_Guid()
        {
            return guid;
        }
        public string Get_Shape()
        {
            return shape;
        }
        public Vector3 Get_Position()
        {
            return Operations.FromAgxVec3(agx_Object.getLocalPosition());
        }
        public double Get_Size()
        {
            return size * 2;//Size in unity is 2 times bigger.
        }
        public Vector3 Get_Rotation()
        {
            return Operations.FromAgxQuat(agx_Object.getLocalRotation()).eulerAngles();
        }
        public Quaternion Get_QuatRotation()
        {
            return Operations.FromAgxQuat(agx_Object.getLocalRotation());
        }
        public double Get_Mass()
        {
            return (double)agx_Object.getMassProperties().getMass();
        }

        /// Material:
        public string Get_MateriaName()
        {
            return materialName;
        }

        ///Simulation:
        public void AddToSim()
        {
            Agx_Simulation.sim_Instance.add(agx_Object);
        }

    }

    public static class Agx_Simulation
    {
        public static agxSDK.Simulation sim_Instance;

        public static void Start(double dt)
        {
            agx.agxSWIG.init();
            sim_Instance = new agxSDK.Simulation();//Initialize the simulation
            sim_Instance.setUniformGravity(new agx.Vec3(0, -9.80665f, 0));//set gravity in Y direction
            sim_Instance.getDynamicsSystem().getTimeGovernor().setTimeStep(dt);// set timestep
        }
        public static void StepForward()
        {
            sim_Instance.stepForward();
        }
        public static void Stop()
        {
            agx.agxSWIG.shutdown();
        }
        public static void AddContactMaterial(string a, string b, double restitution, double friction, double youngsModulus)
        {
            var material_A = new agx.Material(a);
            var material_B = new agx.Material(b);
            var contact_material = new agx.ContactMaterial(material_A, material_B);
            contact_material.setFrictionCoefficient(restitution);
            contact_material.setRestitution(friction);
            contact_material.setYoungsModulus(youngsModulus);
            sim_Instance.add(material_A);
            sim_Instance.add(material_B);
            sim_Instance.add(contact_material);
        }
    }

    public class Agx_Scene
    {

        Guid guid;
        agx.RigidBody terrain;


        /*--------------------------------------------------Creating terrain--------------------------------------------------*/
        //public void Create_Terrain(Guid guid, string heightmap, Vector3 position, string materialName, double restitution, double friction, double height)
        public Agx_Scene(Guid guid, List<Vector3> vertices, List<int> triangles, Vector3 position, string materialName, double height)
        {
            this.guid = guid;

            //AgX:
            agx.Vec3Vector agx_vertices = new agx.Vec3Vector();
            agx.UInt32Vector agx_indices = new agx.UInt32Vector();
            for (int i = 0; i < vertices.Count; i++)
            {
                agx_vertices.Add(Operations.ToAgxVec3(vertices[i]));
            }
            for (int i = 0; i < triangles.Count; i++)
            {
                agx_indices.Add((uint)triangles[i]);
            }
            terrain = new agx.RigidBody();

            //uint optionsMask = (uint)agxCollide.Trimesh.TrimeshOptionsFlags.TERRAIN;
            var terrain_trimesh = new agxCollide.Trimesh(agx_vertices, agx_indices, "handmade terrain");//, optionsMask, height);

            var geometry = new agxCollide.Geometry();
            geometry.add(terrain_trimesh);
            geometry.setMaterial(new agx.Material(materialName));

            terrain.add(geometry);
            terrain.setMotionControl(agx.RigidBody.MotionControl.STATIC);
            terrain.setLocalPosition(Operations.ToAgxVec3(position));//move right and -height for global 0

            ///Adds terrain to simulation
            //simulation.add(terrain);
            Agx_Simulation.sim_Instance.add(terrain);

        }
    }



    class Operations
    {
        /*-----------------------------------------------Mathematical operations----------------------------------------------*/
        /**----------------------------------------From agx.Vec3 to UnityEngine.Vector3---------------------------------------*/
        public static Vector3 FromAgxVec3(agx.Vec3 vec3)
        {
            return new Vector3((double)vec3.x, (double)vec3.y, (double)vec3.z);
        }
        /**----------------------------------------From UnityEngine.Vector3 to agx.Vec3---------------------------------------*/
        public static agx.Vec3 ToAgxVec3(Vector3 vector3)
        {
            return new agx.Vec3(vector3.x, vector3.y, vector3.z);
        }
        /**---------------------------------------From agx.Quat to UnityEngine.Quaternion-------------------------------------*/
        public static Quaternion FromAgxQuat(agx.Quat quat)//wrong
        {
            return new Quaternion((double)quat.x, (double)quat.y, (double)quat.z, (double)quat.w);
        }
        public static agx.Vec3Vector ToAgxVec3Vector(Vector3[] vector3)
        {
            agx.Vec3Vector vec3 = vector3.Count() > 0 ? new agx.Vec3Vector(vector3.Count()) : new agx.Vec3Vector();

            for (int i = 0; i < vector3.Count(); i++)
            {
                vec3.Add(new agx.Vec3(vector3[i].x, vector3[i].y, vector3[i].z));
            }
            return vec3;
        }
        public static agx.UInt32Vector ToAgxIntVector(int[] integers)
        {
            agx.UInt32Vector intVec = integers.Count() > 0 ? new agx.UInt32Vector(integers.Count()) : new agx.UInt32Vector();

            for (int i = 0; i < integers.Count(); i++)
            {
                intVec.Add((uint)integers[i]);
            }
            return intVec;
        }
    }
}