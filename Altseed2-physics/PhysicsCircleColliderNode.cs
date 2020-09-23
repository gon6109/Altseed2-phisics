using System;
using System.Collections.Generic;
using Altseed2;
using Box2DX.Collision;
using Box2DX.Dynamics;

namespace Altseed2.Physics
{
    /// <summary>
    /// 物理対応円
    /// </summary>
    public class PhysicsCircleColliderNode : PhysicsColliderNode
    {
        CircleDef b2CircleDef;
        private float radius;

        /// <summary>
        /// 半径
        /// </summary>
        public float Radius
        {
            get => radius;
            set
            {
                radius = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 初期化
        /// </summary>
        /// <param name="shapeType">物理形状タイプ</param>
        /// <param name="world">登録するワールド</param>
        public PhysicsCircleColliderNode(World world) : base(world)
        {
            b2CircleDef = new CircleDef();
            B2Body = World.B2World.CreateBody(b2BodyDef);
            B2Body.CreateFixture(b2CircleDef);
        }

        protected override void Reset()
        {
            if (B2Body != null)
            {
                World.B2World.DestroyBody(B2Body);
            }
            b2BodyDef = new BodyDef();
            b2CircleDef = new CircleDef();
            b2BodyDef.Angle = Angle / 180.0f * 3.14f;
            b2BodyDef.Position = Position.ToB2Vector();
            b2CircleDef.Radius = radius / (float)PhysicsExtension.PixcelPerMeter;
            b2CircleDef.Density = Density;
            b2CircleDef.Restitution = Restitution;
            b2CircleDef.Friction = Friction;
            b2CircleDef.IsSensor = IsSensor;
            b2CircleDef.Filter = new FilterData()
            {
                GroupIndex = GroupIndex,
                CategoryBits = CategoryBits,
                MaskBits = MaskBits
            };
            B2Body = World.B2World.CreateBody(b2BodyDef);
            B2Body.CreateFixture(b2CircleDef);

            B2Body.SetMassFromShapes();
        }
    }
}
