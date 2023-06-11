
using System.Threading;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public abstract class UnitySubAndPub<T> : MonoBehaviour where T : Message
    {
        public string PubTopic;
        public string SubTopic;
        private string publicationId;

        private RosConnector rosConnector;
        public float TimeStep;
        private readonly int SecondsTimeout = 1;

        protected virtual void Start()
        {
            rosConnector = GetComponent<RosConnector>();
            new Thread(Subscribe).Start();
            publicationId = rosConnector.RosSocket.Advertise<T>(PubTopic);
        }

        protected void Publish(T message)
        {
            rosConnector.RosSocket.Publish(publicationId, message);
        }

        private void Subscribe()
        {

            if (!rosConnector.IsConnected.WaitOne(SecondsTimeout * 1000))
                Debug.LogWarning("Failed to subscribe: RosConnector not connected");

            rosConnector.RosSocket.Subscribe<T>(SubTopic, ReceiveMessage, (int)(TimeStep * 1000)); // the rate(in ms in between messages) at which to throttle the topics
        }

        protected abstract void ReceiveMessage(T message);

    }
}