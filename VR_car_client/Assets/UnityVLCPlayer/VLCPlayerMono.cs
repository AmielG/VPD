using System;
using UnityEngine;

namespace bosqmode.libvlc
{
    public class VLCPlayerMono : MonoBehaviour
    {
        [SerializeField]
        private UnityEngine.UI.RawImage m_rawImage;

        [SerializeField]
        private string url = "rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov";

        [SerializeField]
        [Min(0)]
        [Tooltip("Output resolution width, can be left 0 for automatic scaling")]
        private int width = 480;

        [SerializeField]
        [Min(0)]
        [Tooltip("Output resolution height, can be left 0 for automatic scaling")]
        private int height = 256;

        [Tooltip("Whether to automatically adjust the rawImage's scale to fit the aspect ratio")]
        [SerializeField]
        private bool autoscaleRawImage = true;

        [SerializeField]
        [Tooltip("Mute")]
        private bool mute = true;

        private Texture2D tex;
        private VLCPlayer player;
        private Material skybox360;
        private static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
        private double thetaVFrameRate = 29.9703f;
        private double frameTime = 0f;
        private const double frameDelay = 1000.0f;
        TimeSpan timeSpan;

        private void Start()
        {
            skybox360 = RenderSettings.skybox;
            skybox360.SetTextureScale("_MainTex", new Vector2(-1, -1));
            DynamicGI.UpdateEnvironment();
            player = new VLCPlayer(width, height, url, !mute);

            timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            frameTime = timeSpan.TotalMilliseconds - frameDelay;
        }

        private void Update()
        {
            byte[] img;
            if (player != null && player.CheckForImageUpdate(out img))
            {
                if (tex == null)
                {
                    if ((width <= 0 || height <= 0) && player.VideoTrack != null)
                    {
                        width = (int)player.VideoTrack.Value.i_width;
                        height = (int)player.VideoTrack.Value.i_height;
                    }

                    if (width > 0 && height > 0)
                    {
                        tex = new Texture2D(width, height, TextureFormat.RGB24, false, false);
                        skybox360.SetTexture("_MainTex", tex);
                    }
                }
                else
                {
                    tex.LoadRawTextureData(img);
                    tex.Apply(false);

                    timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
                    frameTime = timeSpan.TotalMilliseconds - frameDelay;
                }
            }
            else
            {
                timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
                Debug.Log("frameDelay: " + (timeSpan.TotalMilliseconds - frameTime));
            }
        }

        public double getFrameDelay()
        {
            timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            return timeSpan.TotalMilliseconds - frameTime;
        }

        private void OnDestroy()
        {
            player?.Dispose();
        }
    }
}