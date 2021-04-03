using System;
using System.Collections.Generic;
using Altseed2;
using Box2DX.Dynamics;

namespace Altseed2.Physics
{
    /// <summary>
    /// 物理対応四角形
    /// </summary>
    public class PhysicsRectangleColliderNode : PhysicsColliderNode
    {
        PolygonDef b2PolygonDef;
        private Vector2F rectangleSize;

        /// <summary>
        /// サイズ
        /// </summary>
        public Vector2F RectangleSize
        {
            get => rectangleSize;
            set
            {
                rectangleSize = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 初期化
        /// </summary>
        /// <param name="world">登録するワールド</param>
        public PhysicsRectangleColliderNode(World world) : base(world)
        {
            b2PolygonDef = new PolygonDef();
            b2PolygonDef.SetAsBox(1 / 2.0f, 1 / 2.0f);
            B2Body = World.B2World.CreateBody(b2BodyDef);
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
            b2BodyDef.Position = Position.ToB2Vector();
            b2BodyDef.Angle = MathHelper.DegreeToRadian(Angle);

            b2PolygonDef = new PolygonDef();
            var temp = RectangleSize.ToB2Vector();
            b2PolygonDef.SetAsBox(temp.X / 2, temp.Y / 2, (RectangleSize / 2 - CenterPosition).ToB2Vector(), 0);
            b2PolygonDef.IsSensor = IsSensor;

            b2PolygonDef.Density = Density * (float)(PhysicsExtension.PixcelPerMeter * PhysicsExtension.PixcelPerMeter);
            b2PolygonDef.Restitution = Restitution;
            b2PolygonDef.Friction = Friction;
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
