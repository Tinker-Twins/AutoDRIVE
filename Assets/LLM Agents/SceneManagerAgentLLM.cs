using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using LLMUnity;

public class SceneManagerAgentLLM : MonoBehaviour {

    // AI agent
    public LLMCharacter llmCharacter;
    public InputField inputText;
    public Text outputText;
    public string prompt;

    public AutomobileController automobileController;
    public GameObject sun;
    public GameObject night;
    public WeatherManager weatherManager;
    public GameObject cementPile;
    public GameObject trafficCones;
    public GameObject roadBarriers;
    public GameObject parkedCar;
    public GameObject AGV;
    public GameObject pedestrian;
    public string task;

    private bool elegantReply = true;

    void Start()
    {
        inputText.onSubmit.AddListener(onInputFieldSubmit);
        inputText.interactable = true;
        inputText.Select();
    }

    void onInputFieldSubmit(string message)
    {
        inputText.interactable = false;
        outputText.text = "Processing...";
        prompt = message;
        string full_message = message + " Does the earlier message talk about 'night', 'day', 'fog', 'rain, 'snow', 'cement', 'traffic cones', 'road barriers', 'parked cars', 'mobile robots', 'pedestrians', or 'autonomous vehicle'. Give me only the exact option as the answer (don't print anything else)!";
        _ = llmCharacter.Chat(full_message, HandleReply, ReplyCompleted);
    }

    void HandleReply(string reply){
        // do something with the reply from the model
        // Debug.Log(reply);
        // outputText.text = reply;
        task = reply;
        elegantReply = true;
        outputText.text = "Generating...";
    }

    void HandleReplyUI(string reply){
        // do something with the reply from the model
        // Debug.Log(reply);
        outputText.text = reply;
    }

    void ReplyCompleted(){
        // do something when the reply from the model is complete
        // Debug.Log("The AI replied");
        Debug.Log(task);
        if (task == "night")
        {
            night.SetActive(true);
            sun.SetActive(false);
        }
        else if(task == "day")
        {
            night.SetActive(false);
            sun.SetActive(true);
        }
        else if (task == "fog") weatherManager.weatherPreset = WeatherManager.WeatherPreset.HeavyFog;
        else if (task == "rain") weatherManager.weatherPreset = WeatherManager.WeatherPreset.HeavyRain;
        else if (task == "snow") weatherManager.weatherPreset = WeatherManager.WeatherPreset.HeavySnow;
        else if (task == "cement") cementPile.SetActive(true);
        else if (task == "traffic cones") trafficCones.SetActive(true);
        else if (task == "road barriers") roadBarriers.SetActive(true);
        else if (task == "parked cars") parkedCar.SetActive(true);
        else if (task == "mobile robots") AGV.SetActive(true);
        else if (task == "pedestrians") pedestrian.SetActive(true);
        else if (task == "autonomous vehicle") automobileController.DrivingMode = 1;
        else
        {
            /// Something wrong. `reply` has incorrect float value (possibly other string part as well)
            outputText.text = "Sorry, I didn't quite get that!";
            elegantReply = false;
        }
        if (elegantReply)
        {
            string full_message = prompt + "Say that you have modified the scenario to include " + task + "Answer in one short sentence. This answer need not be the exact same sentence, you can be slightly creative. Don't include any past information.";
            _ = llmCharacter.Chat(full_message, HandleReplyUI);
        }
        inputText.interactable = true;
        inputText.Select();
        inputText.text = "";
    }

    public void SubmitPrompt()
    {
        onInputFieldSubmit(inputText.text);
    }

    public void CancelRequest()
    {
        llmCharacter.CancelRequests();
        ReplyCompleted();
    }

    bool onValidateWarning = true;
    void OnValidate()
    {
        if (onValidateWarning && !llmCharacter.remote && llmCharacter.llm != null && llmCharacter.llm.model == "")
        {
            Debug.LogWarning($"Please select a model in the {llmCharacter.llm.gameObject.name} GameObject!");
            onValidateWarning = false;
        }
    }
}