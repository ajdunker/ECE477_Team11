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
        int numFramesOnUser = 0;
        Boolean enableSkeleton = true;
        int dabHoldCount = 0;

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
                /*source = kinectSensor.AudioSource;
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
                }*/
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
            if (!enableSkeleton)
                return;
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

                    numFramesOnUser += 1;
                    if (numFramesOnUser >= 5 && dabHoldCount == 0)
                    {
                        playerIndex = (playerIndex + 1) % 6;
                        numFramesOnUser = 0;
                        //dabHoldCount = 0;
                    }
                    /*if (playerIndex == 6 && dabHoldCount == 0)
                    {
                        playerIndex = 0;
                        //numFramesOnUser = 0;
                        dabHoldCount = 0;
                    }*/

                    if (skeletonData[playerIndex].TrackingId != 0)
                    {
                        kinectSensor.SkeletonStream.ChooseSkeletons(skeletonData[playerIndex].TrackingId);
                        //dabHoldCount = 0;
                    }
                    else
                    {
                        return;
                    }
                    //Console.WriteLine("player index: " + playerIndex);
                    //Console.WriteLine("skeleton length: " + skeletonLength);
                    Skeleton playerSkeleton = skeletonData[playerIndex];
                    double theta;
                    if (skeletonData[playerIndex] != null)
                    {
                        rightHand = skeletonData[playerIndex].Joints[JointType.HandRight];
                        rightShoulder = skeletonData[playerIndex].Joints[JointType.ShoulderRight];
                        leftHand = skeletonData[playerIndex].Joints[JointType.HandLeft];
                        leftShoulder = skeletonData[playerIndex].Joints[JointType.ShoulderLeft];
                        head = skeletonData[playerIndex].Joints[JointType.Head];
                        //Console.WriteLine("player index: " + playerIndex + " hold count is: " + dabHoldCount);
                        if (rightHand.Position.Y > rightShoulder.Position.Y && leftHand.Position.Y > leftShoulder.Position.Y && rightHand.Position.X < rightShoulder.Position.X && leftHand.Position.X < leftShoulder.Position.X && leftHand.Position.X < rightShoulder.Position.X)
                        {
                            if (dabHoldCount == 30)
                            {
                                System.Diagnostics.Debug.WriteLine("Recognized dab on user: " + playerSkeleton.TrackingId);
                                theta = Math.Abs(Math.Atan(head.Position.X / head.Position.Z) * 180 / Math.PI);
                                enableSkeleton = false;
                                sendUART(head.Position.X, head.Position.Z, theta);
                                System.Threading.Thread.Sleep(10000);
                                enableSkeleton = true;
                                dabHoldCount = 0;
                            }
                            else
                            {
                                dabHoldCount += 1;
                            }
                        }
                        else
                        {
                            //System.Diagnostics.Debug.WriteLine("No gesture recognized for user: " + playerSkeleton.TrackingId);
                            dabHoldCount = 0;
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

            /*sre = new SpeechRecognitionEngine(ri.Id);

            var speechList = new Choices();
            speechList.Add("hello");
            speechList.Add("computer");
            speechList.Add("engineer");
            speechList.Add("this is a test");
            speechList.Add("fridge me a beer");

            var gb = new GrammarBuilder { Culture = ri.Culture };
            gb.Culture = ri.Culture;

            // Specify the culture to match the recognizer in case we are running in a different culture.                                 
            gb.Append(speechList);

            // Create the actual Grammar instance, and then load it into the speech recognizer.
            var g = new Grammar(gb);

            sre.LoadGrammar(g);
            sre.SpeechRecognized += SreSpeechRecognized;
            sre.SpeechRecognitionRejected += SreSpeechRecognitionRejected;

            s = source.Start();

            sre.SetInputToAudioStream(s, new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
            sre.RecognizeAsync(RecognizeMode.Multiple);*/
        }

        protected override void Initialize()
        {
            KinectSensor.KinectSensors.StatusChanged += new EventHandler<StatusChangedEventArgs>(KinectSensors_StatusChanged);
            DiscoverKinectSensor();
            base.Initialize();
        }

        private void sendUART(float xVal, float zVal, double theta)
        {
            char[] portReadBuffer = new char[8];
            SerialPort port = new SerialPort("COM4", 300, Parity.None, 8, StopBits.One);

            int intAngle = (int)(theta/2);
            if (intAngle > 10)
            {
                intAngle = 10 ;
            }
            //Console.WriteLine("intAngle: " + intAngle);
            char charAngle = (char)intAngle;
            int intFirstDistDecimal = Convert.ToInt32((zVal - Math.Truncate(zVal)) * 10);
            char charFirstDistDecimal = (char)intFirstDistDecimal;
            int intFirstDistVal = Convert.ToInt32(Math.Truncate(zVal));
            char charFirstDistVal = (char)intFirstDistVal;

            Console.WriteLine("angle used: " + intAngle);
            Console.WriteLine("distance used: " + intFirstDistVal + "." + intFirstDistDecimal);

            //open the port for communications
            port.Open();

            if (xVal < 0)
            {
                port.Write("A-" + charAngle + "D" + charFirstDistVal + charFirstDistDecimal);
                //Console.WriteLine("sent to micro: A-" + charAngle + "D" + charFirstDistVal + charFirstDistDecimal);
            }
            else
            {
                port.Write("A+" + charAngle + "D" + charFirstDistVal + charFirstDistDecimal);
                //Console.WriteLine("sent to micro: A+" + charAngle + "D" + charFirstDistVal + charFirstDistDecimal);
            }

            port.Close();
            return;
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
