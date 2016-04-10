using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK.Input;
using OpenTK;

namespace Boids
{
    class Camera
    {
        static KeyboardState key;
        static KeyboardState LastKey;
        //Camera position
        public static Vector2 cameraPos = new Vector2(0, 0);
        public static Vector3 fractalRot = new Vector3(0, 0, 0);
        public static Vector2 fractalScale = new Vector2(1, 1);
        static int step = 5;
        public static int selectedTeam = 0;

       public static bool DEBUGDraw = true;
        public static void CheckMove(GameWindow window)
        {
            key = Keyboard.GetState();
            int booster = 1;
            if (key.IsKeyDown(Key.Q))
                window.Close();
            if (key.IsKeyDown(Key.ShiftRight))
                booster = 2;
            if (key.IsKeyUp(Key.ShiftRight))
                booster = 1;
            if (key.IsKeyDown(Key.Keypad5))
                cameraPos.Y -= step * booster;
            else if (key.IsKeyDown(Key.Keypad8))
                cameraPos.Y += step * booster;
            if (key.IsKeyDown(Key.Keypad4))
                cameraPos.X -= step * booster;
            else if (key.IsKeyDown(Key.Keypad6))
                cameraPos.X += step * booster;

            if (key.IsKeyDown(Key.Keypad7))
                fractalRot.Z += MathHelper.DegreesToRadians(1);
            else if (key.IsKeyDown(Key.Keypad9))
                fractalRot.Z -= MathHelper.DegreesToRadians(1);

            if (key.IsKeyDown(Key.Keypad1))
                fractalScale.X += 0.04F;
            else if (key.IsKeyDown(Key.Keypad3))
                fractalScale.X -= 0.04F;

            if(key.IsKeyDown(Key.D) && LastKey.IsKeyUp(Key.D))
            {
                DEBUGDraw = !DEBUGDraw;
            }

            if (key.IsKeyDown(Key.Number1))
                selectedTeam = 0;
            else if (key.IsKeyDown(Key.Number2))
                selectedTeam = 1;
            LastKey = key; 
        }
    }
}
