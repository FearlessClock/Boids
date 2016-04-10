using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
using OpenTK.Audio;
using System.Drawing;

namespace Boids
{
    struct Vertex
    {
        public Vector2 position;
        public Vector2 texCoord;
        public Vector4 color;

        public Color Color
        {
            get
            {
                return Color.FromArgb((int)(255 * color.W), (int)(255 * color.X), (int)(255 * color.Y), (int)(255 * color.Z));
            }
            set
            {
                this.color = new Vector4(value.R / 255f, value.G / 255f, value.B / 255f, value.A / 255f);
            }

        }
        static public int SizeInBytes
        {
            get { return Vector2.SizeInBytes * 2 + Vector4.SizeInBytes; }
        }

        public Vertex(Vector2 position, Vector2 texCoord)
        {
            this.position = position;
            this.texCoord = texCoord;
            this.color = new Vector4(1, 1, 1, 1);
        }

        public override string ToString()
        {
            return position.X + " : " + position.Y;
        }


    }
    class Game
    {
        public GameWindow window;
        Texture2D texture;

        //Start of the vertex buffer
        Vertex[] vertBuffer;
        int VBO; //Vertex buffer object

        uint[] indexBuffer;
        int IBO;    //Index buffer object

        #region Debug buffers
        //Start of the vertex buffer
        Vertex[] DEBUGvertBuffer;
        int DEBUGVBO; //Vertex buffer object

        uint[] DEBUGindexBuffer;
        int DEBUGIBO;    //Index buffer object
        Buffer DEBUGBuffer = new Buffer();
        #endregion

        public Game(GameWindow windowInput)
        {
            window = windowInput;

            window.Load += Window_Load;
            window.RenderFrame += Window_RenderFrame;
            window.UpdateFrame += Window_UpdateFrame;
            window.Closing += Window_Closing;
        }

        List<Boid> boids = new List<Boid>();
        int nmbrOfBoids = 100;
        Buffer buffer;

        Vector2 lastClickPosition = new Vector2(400, 400);
        bool click = false;
        MouseState lastMS;
        Vector2 pointer = new Vector2(400, 400);

        Random rand = new Random();

        private void Window_Load(object sender, EventArgs e)
        {
            texture = ContentPipe.LoadTexture("explo.bmp");

            vertBuffer = new Vertex[0];
            VBO = GL.GenBuffer();

            indexBuffer = new uint[0];
            IBO = GL.GenBuffer();

            DEBUGvertBuffer = new Vertex[0];
            DEBUGVBO = GL.GenBuffer();

            DEBUGindexBuffer = new uint[0];
            DEBUGIBO = GL.GenBuffer();

            for (int i = 0; i < nmbrOfBoids/2; i++)
            {
                boids.Add(new Boid(rand.Next(0,1), new Vector2(200+i, 200), 
                    Vector2.Transform(Vector2.UnitX, Quaternion.FromAxisAngle(new Vector3(0, 0, 1), (i * 360/(nmbrOfBoids/2)) % 360)), 
                    rand.Next(30, 60), rand.Next(10, 20), 
                    window.Width, window.Height, rand, 0.5+rand.NextDouble(), 2+rand.NextDouble(), 1+rand.NextDouble(), 1+rand.NextDouble()));
            }

            for (int i = (int)Math.Floor((double)nmbrOfBoids/2); i < nmbrOfBoids; i++)
            {
                boids.Add(new Boid(1, new Vector2(1000-i*2, 400-i),
                    Vector2.Transform(Vector2.UnitX, Quaternion.FromAxisAngle(new Vector3(0, 0, 1), (i * 360 / (nmbrOfBoids / 2)) % 360)),
                    rand.Next(30, 60), rand.Next(10, 20),
                    window.Width, window.Height, rand, -1 + rand.NextDouble(), 2 + rand.NextDouble(), 1 + rand.NextDouble(), -1+ rand.NextDouble()));
            }
            lastMS = Mouse.GetCursorState();
        }
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {

        }
        private void Window_UpdateFrame(object sender, FrameEventArgs e)
        {
            Camera.CheckMove(window);
            buffer.Empty();
            DEBUGBuffer.Empty();
            AddMouse();
            for (int i = 0; i < boids.Count; i++)
            {
                if (boids[i].isAlive)
                {
                    boids[i].Update(boids.ToArray<Boid>(), lastClickPosition, pointer);
                    boids[i].Draw(ref buffer);
                    boids[i].DebugDraw(ref DEBUGBuffer);
                }
                else
                {
                    boids.RemoveAt(i);
                    /*boids[i] = new Boid(1, new Vector2(1000, 400 + i),
                    Vector2.Transform(Vector2.UnitX, Quaternion.FromAxisAngle(new Vector3(0, 0, 1), (i * 360 / (nmbrOfBoids / 2)) % 360)),
                    rand.Next(30, 60), rand.Next(10, 20),
                    window.Width, window.Height, rand, -1 + rand.NextDouble(), 2 + rand.NextDouble(), 1 + rand.NextDouble(), -1 + rand.NextDouble()));*/
                }

            }
            Boid.time++;
            vertBuffer = buffer.vertBuffer;
            indexBuffer = buffer.indexBuffer;


            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData<Vertex>(BufferTarget.ArrayBuffer, (IntPtr)(Vertex.SizeInBytes * vertBuffer.Length), vertBuffer, BufferUsageHint.StaticDraw);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, IBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, (IntPtr)(sizeof(uint) * (indexBuffer.Length)), indexBuffer, BufferUsageHint.StaticDraw);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
        }

        private void AddMouse()
        {
            MouseState ms = Mouse.GetCursorState();
            Vector2 position = new Vector2((ms.X), ((767-ms.Y)));
            if(ms.LeftButton == ButtonState.Pressed && lastMS.LeftButton == ButtonState.Released)
            {
                lastClickPosition = position;
                click = true;
            }
            else if(ms.LeftButton == ButtonState.Released && lastMS.LeftButton == ButtonState.Pressed)
            {
                pointer = position - lastClickPosition;
                click = false;
            }
            //Create a list to stock the new vertices
            List<Vertex> vertBuffer = new List<Vertex>();
            List<uint> indexBuffer = new List<uint>();
            //Fill them with the old data
            vertBuffer.AddRange(buffer.vertBuffer);
            indexBuffer.AddRange(buffer.indexBuffer);
            //Get the triangle positions
            Vector2 direction = Vector2.UnitX;
            Vector2 tri1 = position;
            Vector2 tri2 = position - direction * 5 - new Vector2(-direction.Y, direction.X).Normalized() * 10;
            Vector2 tri3 = position - direction * 5 - new Vector2(direction.Y, -direction.X).Normalized() * 10;
            //Place the new points into the lists
            vertBuffer.Add(new Vertex(tri1, new Vector2(0, 0)));
            vertBuffer.Add(new Vertex(tri2, new Vector2(0, 1)));
            vertBuffer.Add(new Vertex(tri3, new Vector2(1, 0)));

            indexBuffer.Add((uint)buffer.vertBuffer.Length);
            indexBuffer.Add((uint)buffer.vertBuffer.Length + 1);
            indexBuffer.Add((uint)buffer.vertBuffer.Length + 2);

            //Get the triangle positions
            direction = Vector2.UnitY;
            tri1 = lastClickPosition;
            tri2 = lastClickPosition - direction * 5 - new Vector2(-direction.Y, direction.X).Normalized() * 10;
            tri3 = lastClickPosition - direction * 5 - new Vector2(direction.Y, -direction.X).Normalized() * 10;
            //Place the new points into the lists
            vertBuffer.Add(new Vertex(tri1, new Vector2(0, 0)) {Color = Color.LightGreen });
            vertBuffer.Add(new Vertex(tri2, new Vector2(0, 1)) { Color = Color.LightYellow });
            vertBuffer.Add(new Vertex(tri3, new Vector2(1, 0)) { Color = Color.LightPink });

            indexBuffer.Add((uint)vertBuffer.Count - 3);
            indexBuffer.Add((uint)vertBuffer.Count - 2);
            indexBuffer.Add((uint)vertBuffer.Count - 1);

            if(click)
            {
                //Get the triangle positions
                tri1 = position;
                direction = tri1.Normalized();
                tri2 = lastClickPosition - direction * 14 + new Vector2(-direction.Y, direction.X).Normalized() * 10;
                tri3 = lastClickPosition - direction * 14 + new Vector2(direction.Y, -direction.X).Normalized() * 10;
                //Place the new points into the lists
                vertBuffer.Add(new Vertex(tri1, new Vector2(0, 0)));
                vertBuffer.Add(new Vertex(tri2, new Vector2(0, 1)));
                vertBuffer.Add(new Vertex(tri3, new Vector2(1, 0)));

                indexBuffer.Add((uint)vertBuffer.Count - 3);
                indexBuffer.Add((uint)vertBuffer.Count - 2);
                indexBuffer.Add((uint)vertBuffer.Count - 1);

            }



            //Put the lists back into the buffer
            buffer.vertBuffer = vertBuffer.ToArray<Vertex>();
            buffer.indexBuffer = indexBuffer.ToArray<uint>();
            lastMS = ms;
        }

        private void Window_RenderFrame(object sender, FrameEventArgs e)
        {
            //Clear screen color
            GL.ClearColor(Color.FromArgb(0, 0, 0));
            GL.Clear(ClearBufferMask.ColorBufferBit);

            //Enable color blending, which allows transparency
            GL.Enable(EnableCap.Blend);
            GL.Enable(EnableCap.Texture2D);
            //Blending everything for transparency
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);

            //Create the projection matrix for the scene
            Matrix4 proj = Matrix4.CreateOrthographicOffCenter(0, window.Width, 0, window.Height, 0, 1);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref proj);
            Matrix4 mat = Matrix4.CreateTranslation(Camera.cameraPos.X, Camera.cameraPos.Y, 0);  //Create a translation matrix
            GL.MultMatrix(ref mat);                 //Load the translation matrix into the modelView matrix
            mat = Matrix4.CreateScale(Camera.fractalScale.X, Camera.fractalScale.X, 0);
            GL.MultMatrix(ref mat);                     //Multiply the scale matrix with the modelview matrix

            //Bind the texture that will be used
            GL.BindTexture(TextureTarget.Texture2D, texture.ID);

            //Enable all the different arrays
            GL.EnableClientState(ArrayCap.ColorArray);
            GL.EnableClientState(ArrayCap.VertexArray);
            GL.EnableClientState(ArrayCap.TextureCoordArray);

            //Debug data draw
            if (Camera.DEBUGDraw)
            {
                DEBUG();
                GL.BindBuffer(BufferTarget.ArrayBuffer, DEBUGVBO);
                GL.VertexPointer(2, VertexPointerType.Float, Vertex.SizeInBytes, (IntPtr)0);
                GL.TexCoordPointer(2, TexCoordPointerType.Float, Vertex.SizeInBytes, (IntPtr)(Vector2.SizeInBytes));
                GL.ColorPointer(4, ColorPointerType.Float, Vertex.SizeInBytes, (IntPtr)(Vector2.SizeInBytes * 2));
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, DEBUGIBO);
                GL.DrawElements(PrimitiveType.Triangles, indexBuffer.Length, DrawElementsType.UnsignedInt, 0);
            }
            //Load the vert and index buffers
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.VertexPointer(2, VertexPointerType.Float, Vertex.SizeInBytes, (IntPtr)0);
            GL.TexCoordPointer(2, TexCoordPointerType.Float, Vertex.SizeInBytes, (IntPtr)(Vector2.SizeInBytes));
            GL.ColorPointer(4, ColorPointerType.Float, Vertex.SizeInBytes, (IntPtr)(Vector2.SizeInBytes * 2));
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, IBO);
            GL.DrawElements(PrimitiveType.Triangles, indexBuffer.Length, DrawElementsType.UnsignedInt, 0);

            

            //Flush everything
            GL.Flush();
            //Write the new buffer to the screen
            window.SwapBuffers();
        }

        private void DEBUG()
        {
            vertBuffer = DEBUGBuffer.vertBuffer;
            indexBuffer = DEBUGBuffer.indexBuffer;

            GL.BindBuffer(BufferTarget.ArrayBuffer, DEBUGVBO);
            GL.BufferData<Vertex>(BufferTarget.ArrayBuffer, (IntPtr)(Vertex.SizeInBytes * vertBuffer.Length), vertBuffer, BufferUsageHint.StaticDraw);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, DEBUGIBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, (IntPtr)(sizeof(uint) * (indexBuffer.Length)), indexBuffer, BufferUsageHint.StaticDraw);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
        }
    }
}
