using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Altseed2;
using Box2DX.Dynamics;

namespace Altseed2.Physics
{
    /// <summary>
    /// 物理対応三角形
    /// </summary>
    public class PhysicsTriangleColliderNode : PhysicsColliderNode
    {
        PolygonDef b2PolygonDef;
        private List<Vector2F> vertexes;

        /// <summary>
        /// 初期化
        /// </summary>
        /// <param name="shapeType">物理形状タイプ</param>
        /// <param name="world">登録するワールド</param>
        public PhysicsTriangleColliderNode(World world) : base(world)
        {
            b2PolygonDef = new PolygonDef();
            vertexes = new List<Vector2F>();
            vertexes.Add(new Vector2F(0, -1));
            vertexes.Add(new Vector2F(1, 0));
            vertexes.Add(new Vector2F(0, 1));
            b2PolygonDef.Vertices = vertexes.Select(v => v.ToB2Vector()).ToArray();
            b2PolygonDef.VertexCount = 3;
            B2Body = World.B2World.CreateBody(b2BodyDef);
            B2Body.CreateFixture(b2PolygonDef);
        }

        public Vector2F Point1
        {
            get => vertexes[0];
            set
            {
                vertexes[0] = value;
                IsRequiredReset = true;
            }
        }

        public Vector2F Point2
        {
            get => vertexes[1];
            set
            {
                vertexes[1] = value;
                IsRequiredReset = true;
            }
        }

        public Vector2F Point3
        {
            get => vertexes[2];
            set
            {
                vertexes[2] = value;
                IsRequiredReset = true;
            }
        }

        protected override void Reset()
        {
            if (!IsRegistered)
                return;

            if (B2Body != null)
            {
                World.B2World.DestroyBody(B2Body);
            }

            b2BodyDef = new BodyDef();
            b2PolygonDef = new PolygonDef();
            b2PolygonDef.Vertices = vertexes.SortTriangleVertexes().Select(v => (v - CenterPosition).ToB2Vector()).ToArray();
            b2PolygonDef.VertexCount = 3;

            b2BodyDef.Angle = Angle / 180.0f * 3.14f;
            b2BodyDef.Position = Position.ToB2Vector();

            b2PolygonDef.Density = Density;
            b2PolygonDef.Restitution = Restitution;
            b2PolygonDef.Friction = Friction;
            b2PolygonDef.IsSensor = IsSensor;
            b2PolygonDef.Filter = new FilterData()
            {
                GroupIndex = GroupIndex,
                CategoryBits = CategoryBits,
                MaskBits = MaskBits
            };

            B2Body = World.B2World.CreateBody(b2BodyDef);
            B2Body.CreateFixture(b2PolygonDef);

            B2Body.SetMassFromShapes();
        }
    }
}
