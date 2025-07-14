using System;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class ChangeColor : MonoBehaviour
{
    //[SerializeField] private GameObject prefab; // Drag your BallPrefab here in the Inspector
    static Socket listener;
    private CancellationTokenSource source;
    public ManualResetEvent allDone;
    public Renderer objectRenderer;
    private Color matColor;

    private Vector3 objectPosition;
    private Vector3 objectRotation;

    public static readonly int PORT = 1755;
    public static readonly int WAITTIME = 1;

    public float fx = 1000f;
    public float fy = 1000f;
    public float cx = 640f;
    public float cy = 360f;

    ChangeColor()
    {
        source = new CancellationTokenSource();
        allDone = new ManualResetEvent(false);
    }

    // Start is called before the first frame update
    async void Start()
    {
        objectRenderer = GetComponent<Renderer>();
        await Task.Run(() => ListenEvents(source.Token));   
    }

    // Update is called once per frame
    void Update()
    {
        objectRenderer.material.color = matColor;
        this.gameObject.transform.localPosition = objectPosition;
        Quaternion newRot = new Quaternion();
        newRot = Quaternion.Euler(objectRotation);
        this.gameObject.transform.localRotation = newRot;
    }

    private void ListenEvents(CancellationToken token)
    {

        
        IPHostEntry ipHostInfo = Dns.GetHostEntry(Dns.GetHostName());
        IPAddress ipAddress = ipHostInfo.AddressList.FirstOrDefault(ip => ip.AddressFamily == AddressFamily.InterNetwork);
        IPEndPoint localEndPoint = new IPEndPoint(ipAddress, PORT);

         
        listener = new Socket(ipAddress.AddressFamily, SocketType.Stream, ProtocolType.Tcp);

         
        try
        {
            listener.Bind(localEndPoint);
            listener.Listen(10);

             
            while (!token.IsCancellationRequested)
            {
                allDone.Reset();

                print("Waiting for a connection... host :" + ipAddress.MapToIPv4().ToString() + " port : " + PORT);
                listener.BeginAccept(new AsyncCallback(AcceptCallback),listener);

                while(!token.IsCancellationRequested)
                {
                    if (allDone.WaitOne(WAITTIME))
                    {
                        break;
                    }
                }
      
            }

        }
        catch (Exception e)
        {
            print(e.ToString());
        }
    }

    void AcceptCallback(IAsyncResult ar)
    {  
        Socket listener = (Socket)ar.AsyncState;
        Socket handler = listener.EndAccept(ar);
 
        allDone.Set();
  
        StateObject state = new StateObject();
        state.workSocket = handler;
        handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0, new AsyncCallback(ReadCallback), state);
    }

    void ReadCallback(IAsyncResult ar)
    {
        StateObject state = (StateObject)ar.AsyncState;
        Socket handler = state.workSocket;

        int read = handler.EndReceive(ar);
  
        if (read > 0)
        {
            state.colorCode.Append(Encoding.ASCII.GetString(state.buffer, 0, read));
            handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0, new AsyncCallback(ReadCallback), state);
        }
        else
        {
            if (state.colorCode.Length > 1)
            { 
                string content = state.colorCode.ToString();
                print($"Read {content.Length} bytes from socket.\n Data : {content}");
                SetObject(content);
            }
            handler.Close();
        }
    }
    Vector3 Get3DFromBoundingBox(Vector2 topLeft, Vector2 bottomRight, float depth)
    {
        // Find center of bounding box
        float u = (topLeft.x + bottomRight.x) / 2f;
        float v = (topLeft.y + bottomRight.y) / 2f;

        // Convert to normalized camera space
        float xNorm = (u - cx) / fx;
        float yNorm = (v - cy) / fy;

        // Apply depth to get actual 3D point
        float X = xNorm * depth;
        float Y = yNorm * depth;
        float Z = depth;

        return new Vector3(X, Y, Z);
    }

    private void SetObject(string data)
    {
        string[] parameters = data.Split(',');
        // Estimated depth in meters
        float estimatedDepth = 3.0f;

        string label = parameters[0];
        objectPosition = new Vector3();
        float x1 = float.Parse(parameters[1]); // top-left x
        float y1 = float.Parse(parameters[2]); // top-left y
        float x2 = float.Parse(parameters[3]); // bottom-right x
        float y2 = float.Parse(parameters[4]); // bottom-right y
        //objectPosition.z = float.Parse(parameters[2]) / 255.0f;
        
        // Build the bounding box corners
        Vector2 topLeft = new Vector2(x1, y1);
        Vector2 bottomRight = new Vector2(x2, y2);
        //objectPosition.z = float.Parse(parameters[2]) / 255.0f;
        objectRotation.x = float.Parse(parameters[3]) / 255.0f;
        objectRotation.y = float.Parse(parameters[4]) / 255.0f;
        objectRotation.z = float.Parse(parameters[5]) / 255.0f;
       
        objectPosition = Get3DFromBoundingBox(topLeft, bottomRight, estimatedDepth);
        Debug.Log($"Object [{label}] placed at world position: {objectPosition}");
    }

    //Set color to the Material
    private void SetColors (string data) 
    {
        string[] colors = data.Split(',');
        matColor = new Color()
        {
            r = float.Parse(colors[0]) / 255.0f,
            g = float.Parse(colors[1]) / 255.0f,
            b = float.Parse(colors[2]) / 255.0f,
            a = float.Parse(colors[3]) / 255.0f
        };

    }

    private void OnDestroy()
    {
        source.Cancel();
    }

    public class StateObject
    {
        public Socket workSocket = null;
        public const int BufferSize = 1024;
        public byte[] buffer = new byte[BufferSize];
        public StringBuilder colorCode = new StringBuilder();
    }
}
