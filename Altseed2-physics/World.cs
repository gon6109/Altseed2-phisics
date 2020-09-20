using System;
using System.Collections.Generic;
using System.Linq;
using Altseed2;
using Box2DX.Collision;
using Box2DX.Dynamics;

namespace Altseed2.Physics
{
    /// <summary>
    /// 物理演算を適用させるワールド
    /// </summary>
    public class World
    {
        List<PhysicsColliderNode> physicsCollider;
        CollisionController collisionController;
        public Box2DX.Dynamics.World B2World { get; }

        /// <summary>
        /// 1ステップあたりの時間（秒）
        /// </summary>
        public float TimeStep { get; set; }

        /// <summary>
        /// 速度更新頻度
        /// </summary>
        public int VelocityItetions { get; set; }

        /// <summary>
        /// 位置更新頻度
        /// </summary>
        public int PositionIterations { get; set; }

        /// <summary>
        /// ワールドを初期化
        /// </summary>
        /// <param name="worldRect">適用範囲</param>
        /// <param name="gravity">重力</param>
        public World(RectF worldRect, Vector2F gravity)
        {
            physicsCollider = new List<PhysicsColliderNode>();
            collisionController = new CollisionController(this);
            AABB aabb = new AABB();
            aabb.LowerBound = worldRect.Position.ToB2Vector();
            aabb.UpperBound = (worldRect.Position + worldRect.Size).ToB2Vector();
            B2World = new Box2DX.Dynamics.World(aabb, gravity.ToB2Vector(false), true);
            B2World.SetContactListener(collisionController);
            B2World.SetContactFilter(new ContactFilter());
            TimeStep = 1.0f / 60.0f;
            VelocityItetions = 8;
            PositionIterations = 1;
        }

        public bool GetIsCollided(PhysicsColliderNode shape1, PhysicsColliderNode shape2, out List<Vector2F> points)
        {
            points = new List<Vector2F>();

            Body shape1B2Body = null, shape2B2Body = null;
            shape1B2Body = shape1.B2Body;
            shape2B2Body = shape2.B2Body;

            foreach (var item in collisionController.CollisionShapes)
            {
                if ((item.BodyA == shape1B2Body && item.BodyB == shape2B2Body) || (item.BodyB == shape1B2Body && item.BodyA == shape2B2Body))
                {
                    points = item.Points;
                    return true;
                }
            }
            return false;
        }

        public void Add(PhysicsColliderNode physicsColliderNode)
        {
            physicsCollider.Add(physicsColliderNode);
        }

        public void Remove(PhysicsColliderNode physicsColliderNode)
        {
            if (physicsColliderNode.IsActive)
                B2World.DestroyBody(physicsColliderNode.B2Body);
            physicsCollider.Remove(physicsColliderNode);
        }

        /// <summary>
        /// 物理演算を1ステップ実行する
        /// </summary>
        public void Update()
        {
            B2World.Step(TimeStep, VelocityItetions, PositionIterations);
            foreach (var item in physicsCollider)
            {
                item.SyncB2body();
            }
        }
    }

    public class CollisionData
    {
        public Body BodyA;
        public Body BodyB;
        public List<Vector2F> Points;

        public CollisionData()
        {
            Points = new List<Vector2F>();
        }
    }

    public class CollisionController : ContactListener
    {
        List<CollisionData> collisionShapes;
        World refWorld;

        public List<CollisionData> CollisionShapes => collisionShapes;

        public CollisionController(World world)
        {
            refWorld = world;
            collisionShapes = new List<CollisionData>();
        }

        public void BeginContact(Contact contact)
        {
            if (!contact.AreTouching) return;
            CollisionData temp = new CollisionData();
            temp.BodyA = contact.FixtureA.Body;
            temp.BodyB = contact.FixtureB.Body;

            WorldManifold manifold;
            contact.GetWorldManifold(out manifold);
            for (int i = 0; i < contact.Manifold.PointCount; i++)
            {
                temp.Points.Add(manifold.Points[i].ToAsdVector());
            }

            collisionShapes.Add(temp);
        }

        public void EndContact(Contact contact)
        {
            CollisionData temp = new CollisionData();
            foreach (var item in collisionShapes)
            {
                if (item.BodyA == contact.FixtureA.Body && item.BodyB == contact.FixtureB.Body)
                {
                    temp = item;
                    break;
                }
            }
            collisionShapes.Remove(temp);
        }

        public void PreSolve(Contact contact, Manifold oldManifold)
        {
        }

        public void PostSolve(Contact contact, ContactImpulse impulse)
        {
            CollisionData temp = null;
            foreach (var item in collisionShapes)
            {
                if (item.BodyA == contact.FixtureA.Body && item.BodyB == contact.FixtureB.Body)
                {
                    temp = item;
                    break;
                }
            }
            if (temp == null)
                return;

            temp.Points.Clear();
            WorldManifold worldManifold;
            contact.GetWorldManifold(out worldManifold);
            for (int i = 0; i < contact.Manifold.PointCount; i++)
            {
                temp.Points.Add(worldManifold.Points[i].ToAsdVector());
            }
        }
    }
}
