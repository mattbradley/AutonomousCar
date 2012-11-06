using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using AutonomousCar;
using AutonomousCar.Entities;
using AutonomousCar.PathFinding;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Microsoft.Xna.Framework;
using FarseerPhysics.Dynamics;
using C5;

namespace AutonomousCar.Simulation
{
    /// <summary>
    /// The Mission class represents the parameters defining a mission including a start pose, a goal pose, and a set of obstacles in an envrionment.
    /// </summary>
    public class Mission
    {
        public Pose Start { get; set; }
        public Pose Goal { get; set; }
        public float AStarEpsilon { get; set; }
        public float AStarGridResolution { get; set; }
        public Environment Environment { get; set; }

        public Mission()
        {
            Start = new Pose(0f, 0f, 0f);
            Goal = new Pose(10f, 10f, 0f);
            AStarEpsilon = 1.5f;
            AStarGridResolution = 1f;
            Environment = new Environment();
        }
    }

    /// <summary>
    /// The MissionLoader class loads missions described in a JSON mission file.
    /// </summary>
    public static class MissionLoader
    {
        private static Random random = new Random();

        public static Mission LoadFromJson(string fileName, World world)
        {
            JObject o = (JObject)JToken.ReadFrom(new JsonTextReader(File.OpenText(fileName)));
            Mission m;

            if (o["mission"] != null)
            {
                m = MissionFactory.CreateMission((MissionFactory.MissionType)Enum.Parse(typeof(MissionFactory.MissionType), (string)o["mission"]), world);
            }
            else
            {
                m = new Mission();
                m.Start = parsePose(o["start"]);
                m.Goal = parsePose(o["goal"]);
                m.Environment = parseEnvironment(o["environment"], world, m.Start.Position, m.Goal.Position);

                if (o["epsilon"] != null)
                    m.AStarEpsilon = (float)o["epsilon"];
                if (o["resolution"] != null)
                    m.AStarGridResolution = (float)o["resolution"];
            }
            return m;
        }

        private static Pose parsePose(JToken pose)
        {
            switch (pose.GetType().Name)
            {
                case "JObject":
                    return new Pose(parseValue(pose["x"]), parseValue(pose["y"]), parseValue(pose["o"]));
                case "JArray":
                    JArray p = (JArray)pose;
                    return new Pose(parseValue(p[0]), parseValue(p[1]), parseValue(p[2]));
            }

            return new Pose();
        }

        private static Obstacle parseObstacle(JToken obs, World world)
        {
            switch (obs.GetType().Name)
            {
                case "JArray":
                    JArray ob = (JArray)obs;
                    if (ob.Count >= 5)
                        return new BoxObstacle(world, parseValue(ob[2]), parseValue(ob[3]), new Vector2(parseValue(ob[0]), parseValue(ob[1])), parseValue(ob[4]));
                    else if (ob.Count == 4)
                    {
                        float hw = parseValue(ob[2]);
                        return new BoxObstacle(world, hw, hw, new Vector2(parseValue(ob[0]), parseValue(ob[1])), parseValue(ob[3]));
                    }
                    else if (ob.Count == 3)
                        return new BoxObstacle(world, 2f, 2f, new Vector2(parseValue(ob[0]), parseValue(ob[1])), parseValue(ob[2]));
                    else if (ob.Count == 2)
                        return new BoxObstacle(world, 2f, 2f, new Vector2(parseValue(ob[0]), parseValue(ob[1])));

                    break;
            }

            return new BoxObstacle(world, 2f, 2f, Vector2.Zero);
        }

        private static float parseValue(JToken v)
        {
            switch (v.GetType().Name)
            {
                case "JValue":
                    return (float)v;
                case "JArray":
                    JArray vArray = (JArray)v;
                    float min = (float)vArray[0], max = (float)vArray[1];
                    if (min > max)
                    {
                        float temp = min;
                        min = max;
                        max = temp;
                    }
                    return (float)(min + (random.NextDouble() * (max - min)));
            }

            return 0f;
        }

        private static Environment parseEnvironment(JToken env, World world, Vector2 start, Vector2 destination)
        {
            switch (env.GetType().Name)
            {
                case "JValue":
                    switch ((string)env)
                    {
                        case "random": return new RandomEnvironment(world, 200, start, destination);
                        case "parking": return new ParkingEnvironment(world);
                    }
                    break;
                case "JArray":
                    Environment e = new Environment();
                    addObstacles((JArray)env, world, e);
                    return e;
                case "JObject":
                    Environment e2 = new Environment();
                    if (env["resolution"] != null)
                        e2.GridResolution = (float)env["resolution"];
                    if (env["width"] != null)
                        e2.GridWidth = (float)env["width"];
                    if (env["height"] != null)
                        e2.GridHeight = (float)env["height"];
                    if (env["origin"] != null)
                        e2.GridOrigin = new Vector2((float)env["origin"][0], (float)env["origin"][1]);

                    if (env["bitmap"] != null)
                        parseBitmap((string)env["bitmap"], world, e2);

                    if (env["obstacles"] != null)
                        addObstacles((JArray)env["obstacles"], world, e2);

                    return e2;
            }

            return new Environment();
        }

        private static void addObstacles(JArray env, World world, Environment e)
        {
            foreach (JToken t in env)
            {
                switch (t.GetType().Name)
                {
                    case "JArray":
                        e.Obstacles.Add(parseObstacle(t, world)); break;
                    case "JObject":
                        JObject tObj = (JObject)t;
                        if (t["obstacle"] != null)
                        {
                            if (t["repeat"] != null)
                            {
                                int count = (int)t["repeat"];
                                if (count < 0) count = 0;
                                for (int i = 0; i < count; i++)
                                    e.Obstacles.Add(parseObstacle(t["obstacle"], world));
                            }
                            else
                            {
                                e.Obstacles.Add(parseObstacle(t["obstacle"], world));
                            }
                        }
                        break;
                }
            }
        }

        private static void parseBitmap(string filename, World world, Environment e)
        {
            if (!File.Exists(filename))
                filename = "../../../../../missions/" + filename;

            Bitmap bitmap = new Bitmap(filename);
            e.GridWidth = bitmap.Width * e.GridResolution;
            e.GridHeight = bitmap.Height * e.GridResolution;
            float obstacleSize = e.GridResolution * 1f;

            for (int x = 0; x < bitmap.Width; x++)
            {
                for (int y = 0; y < bitmap.Height; y++)
                {
                    if (bitmap.GetPixel(x, y).GetBrightness() < 0.5f)
                        e.Obstacles.Add(new BoxObstacle(world, obstacleSize, obstacleSize, e.GridOrigin + new Vector2(x, bitmap.Height - y - 1) * e.GridResolution));
                }
            }
        }
    }

    /// <summary>
    /// The MissionFactory class can create basic hard-coded missions.
    /// </summary>
    public static class MissionFactory
    {
        public enum MissionType
        {
            Junkyard,
            ParkingLot
        }

        public static Mission CreateMission(MissionType type, World world)
        {
            switch (type)
            {
                case MissionType.Junkyard:
                    Pose start = new Pose(10f, 10f, 0f);
                    Pose goal = new Pose(140f, 140f, MathHelper.PiOver2);
                    return new Mission
                    {
                        Start = start,
                        Goal = goal,
                        Environment = new RandomEnvironment(world, 200, start.Position, goal.Position),
                        AStarEpsilon = 1.1f
                    };

                case MissionType.ParkingLot:
                    return new Mission
                    {
                        //Start = new Pose(20f, 10f, 0f),
                        //Goal = new Pose(90f, 83.6f, MathHelper.PiOver2),
                        Start = new Pose(140f, 45f, -MathHelper.Pi),
                        Goal = new Pose(90f, 86.34f, -MathHelper.PiOver2),
                        Environment = new ParkingEnvironment(world),
                        AStarEpsilon = 2f
                    };
            }

            return new Mission();
        }
    }
}
