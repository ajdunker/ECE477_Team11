using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using Microsoft.Kinect;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;
using System.IO;
using System.Threading;
using System.IO.Ports;

namespace KinectTest
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class Game1 : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        Texture2D kinectRGBVideo;
        Texture2D hand;
        Texture2D overlay;
        KinectSensor kinectSensor;
        SpriteFont font;
        Joint rightHand;
        Joint rightShoulder;
        Joint leftHand;
        Joint leftShoulder;
        Joint head;
        Vector2 handPosition = new Vector2();
        List<Vector2> positions = new List<Vector2>();
        int playerIndex = 0;
        int test = 1;

        RecognizerInfo ri;
        KinectAudioSource source;
        SpeechRecognitionEngine sre;
        Stream s;

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

            graphics.PreferredBackBufferWidth = 640;
            graphics.PreferredBackBufferHeight = 480;
        }

        void KinectSensors_StatusChanged(object sender, StatusChangedEventArgs e)
        {
            if (this.kinectSensor == e.Sensor)
            {
                if (e.Status == KinectStatus.Disconnected ||
                    e.Status == KinectStatus.NotPowered)
                {
                    this.kinectSensor = null;
                    this.DiscoverKinectSensor();
                }
            }
        }

        private bool InitializeKinect()
        {
            //color stream
            kinectSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            kinectSensor.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinectSensor_ColorFrameReady);

            // Skeleton Stream
            kinectSensor.SkeletonStream.Enable(new TransformSmoothParameters()
            {
                Smoothing = 0.5f,
                Correction = 0.5f,
                Prediction = 0.5f,
                JitterRadius = 0.05f,
                MaxDeviationRadius = 0.04f
            });
            kinectSensor.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(kinectSensor_SkeletonFrameReady);

            try
            {
                kinectSensor.Start();

                // Obtain the KinectAudioSource to do audio capture
                source = kinectSensor.AudioSource;
                source.EchoCancellationMode = EchoCancellationMode.None; // No AEC for this sample
                source.AutomaticGainControlEnabled = false; // Important to turn this off for speech recognition

                ri = GetKinectRecognizer();

                if (ri != null)
                {
                    Console.WriteLine("*** using speech");

                    int wait = 4;
                    while (wait > 0)
                    {
                        wait--;
                        Thread.Sleep(1000);
                    }
                    kinectSensor.Start();
                }
            }
            catch
            {
                return false;
            }
            return true;
        }

        void kinectSensor_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorImageFrame = e.OpenColorImageFrame())
            {
                if (colorImageFrame != null)
                {

                    byte[] pixelsFromFrame = new byte[colorImageFrame.PixelDataLength];

                    colorImageFrame.CopyPixelDataTo(pixelsFromFrame);

                    Color[] color = new Color[colorImageFrame.Height * colorImageFrame.Width];
                    kinectRGBVideo = new Texture2D(graphics.GraphicsDevice, colorImageFrame.Width, colorImageFrame.Height);

                    //Go through each pixel and set the bytes correctly
                    //Each pixel has a Red, Green and Blue
                    int index = 0;
                    for (int y = 0; y < colorImageFrame.Height; y++)
                    {
                        for (int x = 0; x < colorImageFrame.Width; x++, index += 4)
                        {
                            color[y * colorImageFrame.Width + x] = new Color(pixelsFromFrame[index + 2], pixelsFromFrame[index + 1], pixelsFromFrame[index + 0]);
                        }
                    }

                    //Set pixeldata from the ColorImageFrame to a Texture2D
                    kinectRGBVideo.SetData(color);
                }
            }
        }

        void kinectSensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    Skeleton[] skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    int skeletonLength = 0;

                    skeletonFrame.CopySkeletonDataTo(skeletonData);

                    if (!this.kinectSensor.SkeletonStream.AppChoosesSkeletons)
                    {
                        this.kinectSensor.SkeletonStream.AppChoosesSkeletons = true;
                    }

                    foreach (Skeleton skel in skeletonData.Where(s => s.TrackingId != 0))
                        skeletonLength += 1;

                    test += 1;
                    if (test % 5 == 0)
                    {
                        playerIndex += 1;
                    }
                    if (playerIndex == 6)
                    {
                        playerIndex = 0;
                    }

                    if (skeletonData[playerIndex].TrackingId != 0)
                    {
                        kinectSensor.SkeletonStream.ChooseSkeletons(skeletonData[playerIndex].TrackingId);
                    }
                    else
                    {
                        return;
                    }

                    Skeleton playerSkeleton = skeletonData[playerIndex];
                    //System.Diagnostics.Debug.WriteLine("tracking user: " + skeletonData[playerIndex].TrackingId);
                    //System.Diagnostics.Debug.WriteLine("skeleton array: " + skeletonData[0].TrackingState + "  " + skeletonData[1].TrackingState + "  " + skeletonData[2].TrackingState + "  " + skeletonData[3].TrackingState + "  " + skeletonData[4].TrackingState + "  " + skeletonData[5].TrackingState);
                    //foreach (Skeleton playerSkeleton in skeletonData.Where(s => s.TrackingState != SkeletonTrackingState.NotTracked))
                    if (skeletonData[playerIndex] != null)
                    {
                        rightHand = skeletonData[playerIndex].Joints[JointType.HandRight];
                        rightShoulder = skeletonData[playerIndex].Joints[JointType.ShoulderRight];
                        leftHand = skeletonData[playerIndex].Joints[JointType.HandLeft];
                        leftShoulder = skeletonData[playerIndex].Joints[JointType.ShoulderLeft];
                        head = skeletonData[playerIndex].Joints[JointType.Head];
                        //System.Diagnostics.Debug.WriteLine("head position of user: " + playerSkeleton.TrackingId + " is: " + head.Position.X);
                        if (rightHand.Position.Y > rightShoulder.Position.Y && leftHand.Position.Y > leftShoulder.Position.Y && rightHand.Position.X < rightShoulder.Position.X && leftHand.Position.X < leftShoulder.Position.X && leftHand.Position.X < rightShoulder.Position.X)
                        {
                            System.Diagnostics.Debug.WriteLine("Recognized dab on user: " + playerSkeleton.TrackingId);
                        }
                        else
                        {
                            //System.Diagnostics.Debug.WriteLine("No gesture recognized for user: " + playerSkeleton.TrackingId);
                        }

                        //handPosition = new Vector2((((0.5f *rightHand.Position.X) + 0.5f) * (640)), (((-0.5f * rightHand.Position.Y) + 0.5f) * (480)));
                    }
                }
            }
        }

        private void DiscoverKinectSensor()
        {
            foreach (KinectSensor sensor in KinectSensor.KinectSensors)
            {
                if (sensor.Status == KinectStatus.Connected)
                {
                    //Found one, set our sensor to this
                    kinectSensor = sensor;
                    break;
                }
            }

            if (this.kinectSensor == null)
            {
                return;
            }

            //Initialize the found and connected device
            if (kinectSensor.Status == KinectStatus.Connected)
            {
                InitializeKinect();
            }

            sre = new SpeechRecognitionEngine(ri.Id);

            var colors = new Choices();
            colors.Add("red");
            colors.Add("green");
            colors.Add("blue");
            colors.Add("clear");
            colors.Add("fridge me a beer");

            var gb = new GrammarBuilder { Culture = ri.Culture };
            gb.Culture = ri.Culture;

            // Specify the culture to match the recognizer in case we are running in a different culture.                                 
            gb.Append(colors);

            // Create the actual Grammar instance, and then load it into the speech recognizer.
            var g = new Grammar(gb);

            sre.LoadGrammar(g);
            sre.SpeechRecognized += SreSpeechRecognized;
            //sre.SpeechHypothesized += SreSpeechHypothesized;
            sre.SpeechRecognitionRejected += SreSpeechRecognitionRejected;
            //sre.SpeechDetected += SreSpeechDetected;

            s = source.Start();

            sre.SetInputToAudioStream(s, new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
            sre.RecognizeAsync(RecognizeMode.Multiple);
        }

        protected override void Initialize()
        {
            SerialPort port = new SerialPort("COM4", 9600, Parity.None, 8, StopBits.One);

            // Open the port for communications
            port.Open();

            // Write a string
            port.Write("Hello World");

            // Close the port
            port.Close();

            KinectSensor.KinectSensors.StatusChanged += new EventHandler<StatusChangedEventArgs>(KinectSensors_StatusChanged);
            DiscoverKinectSensor();

            base.Initialize();
        }

        private static RecognizerInfo GetKinectRecognizer()
        {
            Func<RecognizerInfo, bool> matchingFunc = r =>
            {
                string value;
                r.AdditionalInfo.TryGetValue("Kinect", out value);
                return "True".Equals(value, StringComparison.InvariantCultureIgnoreCase) && "en-US".Equals(r.Culture.Name, StringComparison.InvariantCultureIgnoreCase);
            };
            return SpeechRecognitionEngine.InstalledRecognizers().Where(matchingFunc).FirstOrDefault();
        }

        private static void SreSpeechRecognitionRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {

            if (e.Result != null)
            {
                Console.WriteLine("*** speech SreSpeechRecognitionRejected");
            }
        }

        private void SreSpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {

            // Console.WriteLine("*** speech recog");
            if (e.Result.Confidence >= 0.80)
            {
                Console.WriteLine("*** setColor asked for " + e.Result.Text + ", with a confidence of: " + e.Result.Confidence.ToString());
            }
            else
            {
                Console.WriteLine("Heard something, but the confidence is too low: " + e.Result.Confidence.ToString());
            }
        }

        protected override void LoadContent()
        {
            spriteBatch = new SpriteBatch(GraphicsDevice);
            kinectRGBVideo = new Texture2D(GraphicsDevice, 1337, 1337);

            //overlay = Content.Load<Texture2D>("overlay");
            //hand = Content.Load<Texture2D>("blue-dot");
        }

        protected override void UnloadContent()
        {
            kinectSensor.Stop();
            kinectSensor.Dispose();
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);
            spriteBatch.Begin();
            spriteBatch.Draw(kinectRGBVideo, new Rectangle(0, 0, 640, 480), Color.White);
            //spriteBatch.Draw(hand, handPosition, Color.White);
            //spriteBatch.Draw(overlay, new Rectangle(0, 0, 640, 480), Color.White);
            //spriteBatch.DrawString(font, connectedStatus, new Vector2(20, 80), Color.White);
            spriteBatch.End();

            base.Draw(gameTime);
        }
    }
}
