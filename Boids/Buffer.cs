using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Boids
{
    struct Buffer
    {
        public Vertex[] vertBuffer;
        public uint[] indexBuffer;
        public Buffer(Vertex[] vert, uint[] index)
        {
            vertBuffer = vert;
            indexBuffer = index;
        }

        public void Empty()
        {
            vertBuffer = new Vertex[0];
            indexBuffer = new uint[0];
        }
    }
}
