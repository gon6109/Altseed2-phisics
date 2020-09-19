using System;
using System.Threading;
using NUnit.Framework;
using Altseed2;
using System.Linq;
using Altseed2.Physics;

namespace Test
{
    [TestFixture]
    public class Physics
    {
        [Test, Apartment(ApartmentState.STA)]
        public void Basic()
        {
            Engine.Initialize("Basic", 800, 600, new Configuration());

            var world = new World(new RectF(-100, -100, 1000, 1000), new Vector2F(0, 10));
            var floor = new RectangleNode();
            floor.RectangleSize = new Vector2F(800, 50);
            Engine.AddNode(floor);

            var floorCollider = new PhysicsRectangleColliderNode(world);
            floorCollider.Position = new Vector2F(0.0f, 550.0f);
            floorCollider.RectangleSize = floor.RectangleSize;
            floorCollider.PhysicsColliderType = PhysicsColliderType.Static;
            floorCollider.Restitution = 0.2f;
            floor.AddChildNode(floorCollider);

            var sprite = new RectangleNode();
            sprite.Position = new Vector2F(400, 80);
            sprite.RectangleSize = new Vector2F(50, 50);
            sprite.CenterPosition = sprite.RectangleSize / 2;
            sprite.Angle = 40;
            Engine.AddNode(sprite);

            var collider = new PhysicsRectangleColliderNode(world);
            collider.CenterPosition = sprite.CenterPosition;
            collider.Position = sprite.Position;
            collider.Angle = sprite.Angle;
            collider.RectangleSize = sprite.RectangleSize;
            collider.PhysicsColliderType = PhysicsColliderType.Dynamic;
            collider.Restitution = 0.2f;
            sprite.AddChildNode(collider);

            var text = new TextNode();
            text.Font = Font.LoadDynamicFont("../TestData/Font/mplus-1m-regular.ttf", 50);
            Engine.AddNode(text);

            int count = 0;
            while (Engine.DoEvents())
            {
                if (count % 30 == 0)
                {
                    var sprite1 = new RectangleNode();
                    sprite1.Position = new Vector2F(400, 80);
                    sprite1.RectangleSize = new Vector2F(50, 50);
                    sprite1.CenterPosition = sprite1.RectangleSize / 2;
                    sprite1.Angle = 40;
                    Engine.AddNode(sprite1);

                    var collider1 = new PhysicsRectangleColliderNode(world);
                    collider1.CenterPosition = sprite1.CenterPosition;
                    collider1.Position = sprite1.Position;
                    collider1.Angle = sprite1.Angle;
                    collider1.RectangleSize = sprite1.RectangleSize;
                    collider1.PhysicsColliderType = PhysicsColliderType.Dynamic;
                    collider1.Restitution = 1f;
                    sprite1.AddChildNode(collider1);

                    var triangle = new TriangleNode();
                    triangle.Position = new Vector2F(300, 80);
                    triangle.Point1 = new Vector2F(50, 50);
                    triangle.Point2 = new Vector2F(100, 50);
                    triangle.Point3 = new Vector2F(0, 0);
                    triangle.Angle = 40;
                    Engine.AddNode(triangle);

                    var collider2 = new PhysicsPolygonColliderNode(world);
                    collider2.Position = triangle.Position;
                    collider2.Angle = triangle.Angle;
                    collider2.AddVertex(new Vector2F(50, 50));
                    collider2.AddVertex(new Vector2F(0, 0));
                    collider2.AddVertex(new Vector2F(100, 50));
                    collider2.PhysicsColliderType = PhysicsColliderType.Dynamic;
                    collider2.Restitution = 0f;
                    triangle.AddChildNode(collider2);
                }

                if (count++ > 4000) break;
                //text.Text = collider.Velocity.ToString();
                world.Update();
                Engine.Update();
            }

            Engine.Terminate();
        }

    }
}
