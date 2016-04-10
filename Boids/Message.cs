using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Boids
{
    enum MessageType {Attack, Defend, Run };
    struct Message
    {
        MessageType message;
        public MessageType GetMessage
        {
            get { return message; }
            private set { message = value; }
        }
        int value;
        public int Value
        {
            get { return value; }
            private set {; }
        }

        int callTime;
        public int CallTime
        {
            get { return callTime; }
            private set {; }
        }
        public Message(MessageType messType, int value, int call)
        {
            message = messType;
            this.value = value;
            callTime = call;
        }
    }
}
