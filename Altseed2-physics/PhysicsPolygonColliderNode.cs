using System;
using System.Collections.Generic;
using System.Linq;
using Altseed2;
using Box2DX.Dynamics;
using Math = System.Math;

namespace Altseed2.Physics
{
    /// <summary>
    /// 物理対応多角形
    /// </summary>
    public class PhysicsPolygonColliderNode : PhysicsColliderNode
    {
        List<PolygonDef> b2PolygonDefs;
        private List<Vector2F> vertexes;

        /// <summary>
        /// 初期化
        /// </summary>
        /// <param name="world">登録するワールド</param>
        public PhysicsPolygonColliderNode(World world) : base(world)
        {

            b2PolygonDefs = new List<PolygonDef>();
            vertexes = new List<Vector2F>();

            B2Body = World.B2World.CreateBody(b2BodyDef);
            foreach (var item in b2PolygonDefs) B2Body.CreateFixture(item);
        }

        /// <summary>
        /// 多角形を構成する頂点を追加する
        /// </summary>
        /// <param name="vertex">追加する頂点</param>
        public void AddVertex(Vector2F vertex)
        {
            if (vertexes.FindAll(obj => obj == vertex).Count != 0) return;
            vertexes.Add(vertex);
            Reset();
        }

        /// <summary>
        /// 多角形を構成する頂点を全て削除する
        /// </summary>
        public void ClearVertexes()
        {
            vertexes.Clear();
            Reset();
        }

        protected override void Reset()
        {
            if (vertexes.Count < 3) return;

            if (B2Body != null)
            {
                World.B2World.DestroyBody(B2Body);
            }

            b2BodyDef = new BodyDef();

            b2BodyDef.Angle = Angle / 180.0f * 3.14f;
            b2BodyDef.Position = Position.ToB2Vector();

            b2PolygonDefs = DivideToTriangles(vertexes.Select(v => v - CenterPosition).ToList());
            B2Body = World.B2World.CreateBody(b2BodyDef);
            foreach (var item in b2PolygonDefs)
            {
                FixtureDef fixtureDef = new FixtureDef();

                B2Body.CreateFixture(item);
            }

            B2Body.SetMassFromShapes();
        }

        List<PolygonDef> DivideToTriangles(List<Vector2F> argVertexes)
        {
            if (argVertexes.Count < 3) return null;

            List<PolygonDef> result = new List<PolygonDef>();
            if (argVertexes.Count == 3)
            {
                result.Add(CreatePolygonShape(argVertexes[0], argVertexes[1], argVertexes[2]));
                return result;
            }

            Vector2F root = new Vector2F();
            foreach (var item in argVertexes)
            {
                if (root.Length < item.Length) root = item;
            }

            Vector2F next1, next2;
            next1 = argVertexes.IndexOf(root) != argVertexes.Count - 1 ? argVertexes[argVertexes.IndexOf(root) + 1] : argVertexes[0];
            next2 = argVertexes.IndexOf(root) != 0 ? argVertexes[argVertexes.IndexOf(root) - 1] : argVertexes[argVertexes.Count - 1];

            float cross = Vector2F.Cross(next1 - root, next2 - root);
            while (true)
            {
                bool isDivideble = true;
                foreach (var item in argVertexes)
                {
                    if (IsContainAtTriangle(root, next1, next2, item)) isDivideble = false;
                }

                if (!isDivideble)
                {
                    do
                    {
                        root = argVertexes.IndexOf(root) != argVertexes.Count - 1 ? argVertexes[argVertexes.IndexOf(root) + 1] : argVertexes[0];
                        next1 = argVertexes.IndexOf(next1) != argVertexes.Count - 1 ? argVertexes[argVertexes.IndexOf(next1) + 1] : argVertexes[0];
                        next2 = argVertexes.IndexOf(next2) != argVertexes.Count - 1 ? argVertexes[argVertexes.IndexOf(next2) + 1] : argVertexes[0];
                    } while (Math.Sign(cross) != Math.Sign(Vector2F.Cross(next1 - root, next2 - root)));
                }
                else break;
            }

            result.Add(CreatePolygonShape(root, next1, next2));
            List<Vector2F> remain = new List<Vector2F>(argVertexes);
            remain.Remove(root);
            result.AddRange(DivideToTriangles(remain));
            return result;
        }

        bool IsContainAtTriangle(Vector2F vertex1, Vector2F vertex2, Vector2F vertex3, Vector2F vector)
        {
            float c1 = Vector2F.Cross(vertex2 - vertex1, vector - vertex2);
            float c2 = Vector2F.Cross(vertex3 - vertex2, vector - vertex3);
            float c3 = Vector2F.Cross(vertex1 - vertex3, vector - vertex1);

            if (c1 > 0 && c2 > 0 && c3 > 0 || c1 < 0 && c2 < 0 && c3 < 0) return true;

            return false;
        }

        PolygonDef CreatePolygonShape(Vector2F vertex1, Vector2F vertex2, Vector2F vertex3)
        {
            PolygonDef b2PolygonDef = new PolygonDef();
            var vertex = new List<Vector2F>();
            vertex.Add(vertex1);
            vertex.Add(vertex2);
            vertex.Add(vertex3);
            b2PolygonDef.Vertices = vertex.SortTriangleVertexes().Select(v => v.ToB2Vector()).ToArray();
            b2PolygonDef.VertexCount = b2PolygonDef.Vertices.Length;
            b2PolygonDef.Density = Density;
            b2PolygonDef.Restitution = Restitution;
            b2PolygonDef.Friction = Friction;
            b2PolygonDef.Filter = new FilterData()
            {
                GroupIndex = GroupIndex,
                CategoryBits = CategoryBits,
                MaskBits = MaskBits
            };
            return b2PolygonDef;
        }
    }
}
