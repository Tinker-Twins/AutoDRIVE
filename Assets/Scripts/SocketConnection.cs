using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SocketConnection : MonoBehaviour
{
    /*
    This script toggles the socket connection with server.
    */

    public GameObject Socket;
    public GameObject SocketIO;

    private bool Enabled = false;

    public void ToggleSocketConnection()
    {
        Enabled = !Enabled;
        //Debug.Log(Enabled);
        
        if(Enabled)
        {
            Socket.SetActive(true);
            SocketIO.SetActive(true);
        }
        else
        {
            Socket.SetActive(false);
            SocketIO.SetActive(false);
        }
    }
}