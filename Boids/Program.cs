using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;

namespace Boids
{
    class Program
    {
        static void Main(string[] args)
        {
            GameWindow window = new GameWindow(1366, 768, OpenTK.Graphics.GraphicsMode.Default, "Boids", GameWindowFlags.Fullscreen);
            Game game = new Game(window);

            window.Run();
        }
    }
}
