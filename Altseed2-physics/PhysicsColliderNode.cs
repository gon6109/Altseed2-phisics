using System;
using System.Collections.Generic;
using Altseed2;
using Box2DX.Common;
using Box2DX.Dynamics;
using static Box2DX.Dynamics.Body;

namespace Altseed2.Physics
{
    /// <summary>
    /// Altseed-Box2D変換系
    /// </summary>
    public static class PhysicsExtension
    {
        public static double PixcelPerMeter = 100;

        public static Vector2F ToAsdVector(this Vec2 b2vector, bool isDistance = true)
        {
            if (!isDistance)
                return new Vector2F(b2vector.X, b2vector.Y);

            return new Vector2F((float)(b2vector.X * PixcelPerMeter), (float)(b2vector.Y * PixcelPerMeter));
        }

        public static Vec2 ToB2Vector(this Vector2F asdVector, bool isDistance = true)
        {
            if (!isDistance)
                return new Vec2(asdVector.X, asdVector.Y);

            return new Vec2((float)(asdVector.X / PixcelPerMeter), (float)(asdVector.Y / PixcelPerMeter));
        }

        public static IEnumerable<Vector2F> SortTriangleVertexes(this List<Vector2F> vertexes)
        {
            if ((vertexes[2] - vertexes[0]).Degree.CompareTo((vertexes[1] - vertexes[0]).Degree) > 0)
                return new List<Vector2F>() { vertexes[0], vertexes[1], vertexes[2] };
            else
                return new List<Vector2F>() { vertexes[0], vertexes[2], vertexes[1] };
        }
    }

    /// <summary>
    /// 物理対応図形
    /// </summary>
    public abstract class PhysicsColliderNode : Node
    {
        private float angle;
        private float density;
        private float restitution;
        private float friction;
        short groupIndex;
        ushort categoryBits;
        ushort maskBits;
        private Vector2F position;
        private Vector2F centerPosition;
        private PhysicsColliderType physicsColliderType;

        internal BodyDef b2BodyDef;
        private bool isSensor;

        internal World World { get; }

        internal Body B2Body { get; protected private set; }

        /// <summary>
        /// 座標
        /// </summary>
        public Vector2F Position
        {
            get => position;
            set
            {
                position = value;
                if (B2Body != null)
                    B2Body.SetPosition(position.ToB2Vector());
                else
                {
                    IsRequiredReset = true;
                }
            }
        }

        /// <summary>
        /// 回転の中心
        /// </summary>
        public Vector2F CenterPosition
        {
            get => centerPosition;
            set
            {
                centerPosition = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 角度
        /// </summary>
        public float Angle
        {
            get => angle;
            set
            {
                angle = value;
                if (B2Body != null)
                    B2Body.SetAngle(MathHelper.DegreeToRadian(angle));
                else
                {
                    IsRequiredReset = true;
                }
            }
        }

        /// <summary>
        /// 密度
        /// </summary>
        public float Density
        {
            get => PhysicsColliderType == PhysicsColliderType.Dynamic ? density : 0;
            set
            {
                density = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 反発係数
        /// </summary>
        public float Restitution
        {
            get => restitution;
            set
            {
                restitution = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 摩擦係数
        /// </summary>
        public float Friction
        {
            get => friction;
            set
            {
                friction = value;
                IsRequiredReset = true;
            }
        }

        public bool IsSensor
        {
            get => isSensor;
            set
            {
                isSensor = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 速度
        /// </summary>
        public Vector2F Velocity
        {
            get
            {
                if (!IsActive) return new Vector2F();
                return B2Body.GetLinearVelocity().ToAsdVector();
            }
            set
            {
                if (!IsActive) return;
                B2Body.SetLinearVelocity(value.ToB2Vector());
            }
        }

        /// <summary>
        /// 角速度
        /// </summary>
        public float AngularVelocity
        {
            get
            {
                if (!IsActive) return 0;
                return (float)B2Body.GetAngularVelocity() * 180.0f / 3.14f;
            }
            set
            {
                if (!IsActive) return;
                B2Body.SetAngularVelocity(value / 180.0f * 3.14f);
            }
        }

        /// <summary>
        /// 衝突判定グループ
        /// </summary>
        public short GroupIndex
        {
            get => groupIndex;
            set
            {
                groupIndex = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 衝突判定カテゴリー
        /// </summary>
        public ushort CategoryBits
        {
            get => categoryBits;
            set
            {
                categoryBits = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// どのカテゴリーと衝突するか
        /// </summary>
        public ushort MaskBits
        {
            get => maskBits;
            set
            {
                maskBits = value;
                IsRequiredReset = true;
            }
        }

        /// <summary>
        /// 物理シミュレーションをするか否か
        /// </summary>
        public bool IsActive
        {
            get
            {
                return B2Body != null;
            }
            set
            {
                if (value)
                {
                    if (IsActive == false) Reset();
                }
                else
                {
                    if (IsActive == true) B2Body.GetWorld().DestroyBody(B2Body);
                    B2Body = null;
                }
            }
        }

        /// <summary>
        /// 親のトランスフォームを同期させるか
        /// </summary>
        public bool IsSyncToParentTransform { get; set; }

        internal bool IsRequiredReset { get; set; }

        /// <summary>
        /// コライダーの種類
        /// </summary>
        public PhysicsColliderType PhysicsColliderType
        {
            get => physicsColliderType;
            set
            {
                physicsColliderType = value;
                IsRequiredReset = true;
            }
        }

        public PhysicsColliderNode(World world)
        {
            density = 1.0f;
            restitution = 0.3f;
            angle = 0.0f;
            groupIndex = 0;
            categoryBits = 0x0001;
            maskBits = 0xffff;
            World = world;

            IsSyncToParentTransform = true;
        }

        protected override void OnAdded()
        {
            base.OnAdded();
            Reset();
            World.Add(this);
        }

        protected override void OnUpdate()
        {
            if (IsRequiredReset)
            {
                Reset();
                IsRequiredReset = false;
            }

            base.OnUpdate();
        }

        public void SyncB2body()
        {
            if (!IsActive) return;

            position = B2Body.GetPosition().ToAsdVector();
            angle = MathHelper.RadianToDegree(B2Body.GetAngle());

            if (IsSyncToParentTransform && Parent is TransformNode transformNode)
            {
                transformNode.Position = B2Body.GetPosition().ToAsdVector();
                transformNode.Angle = MathHelper.RadianToDegree(B2Body.GetAngle());
            }
        }

        /// <summary>
        /// 初期化
        /// </summary>
        protected virtual void Reset()
        {
        }

        /// <summary>
        /// 力を加える
        /// </summary>
        /// <param name="vector">力を加える方向</param>
        /// <param name="position">力を加えるローカル位置</param>
        public void SetForce(Vector2F vector, Vector2F position)
        {
            if (!IsActive) return;
            B2Body.ApplyForce(vector.ToB2Vector(), (Position + position).ToB2Vector());
        }

        /// <summary>
        /// 衝撃を加える
        /// </summary>
        /// <param name="vector">衝撃を加える方向</param>
        /// <param name="position">衝撃を加えるローカル位置</param>
        public void SetImpulse(Vector2F vector, Vector2F position)
        {
            if (!IsActive) return;
            B2Body.ApplyImpulse(vector.ToB2Vector(), (Position + position).ToB2Vector());
        }

        /// <summary>
        /// 衝突判定
        /// </summary>
        /// <param name="shape">衝突判定対象</param>
        public bool GetIsCollidedWith(PhysicsColliderNode shape)
        {
            if (!IsActive) return false;
            return World.GetIsCollided(this, shape, out _);
        }

        /// <summary>
        /// 衝突判定
        /// </summary>
        /// <param name="shape">衝突判定対象</param>
        /// <param name="points">衝突点</param>
        public bool GetIsCollidedWith(PhysicsColliderNode shape, out List<Vector2F> points)
        {
            if (!IsActive)
            {
                points = new List<Vector2F>();
                return false;
            }
            return World.GetIsCollided(this, shape, out points);
        }

        protected override void OnRemoved()
        {
            World.Remove(this);
            base.OnRemoved();
        }
    }

    /// <summary>
    /// 物理形状タイプ
    /// </summary>
    public enum PhysicsColliderType
    {
        Static = BodyType.Static,
        //Kinematic = BodyType.Kinematic,
        Dynamic = BodyType.Dynamic,
    }
}