using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class WeatherManager : MonoBehaviour
{
    /*
    This script manages weather. The weather can be chosen based
    on one of the presets or completely customized based on the
    intensities of various elements.
    */

    public VolumeProfile VolumeProfile; // HDRP volume profile
    public GameObject Rain; // Rain gameobject
    public GameObject SubRain; // Sub-rain gameobject
    public ParticleSystem RainParticles; // Rain particle system
    public ParticleSystem RainSubParticles; // Rain subparticle system
    public GameObject Snow; // Snow gameobject
    public ParticleSystem SnowParticles; // Snow particle system
    public Transform EgoVehicle; // Ego vehicle transform (used to set precipitation transform)
    public enum WeatherPreset {Custom, Sunny, Cloudy, LightFog, HeavyFog, LightRain, HeavyRain, LightSnow, HeavySnow}; // Choice of weather conditions
    public WeatherPreset weatherPreset = WeatherPreset.Custom; // Set weather preset
    public float CloudIntensity = 0.0f;
    public float FogIntensity = 0.0f;
    public float RainIntensity = 0.0f;
    public float SnowIntensity = 0.0f;
    public bool OptimizedWeather = false;

    // Update is called once per frame
    void Update()
    {
        if (!VolumeProfile.TryGet<VolumetricClouds>(out var clouds))
        {
            clouds = VolumeProfile.Add<VolumetricClouds>(false);
        }
        if (!VolumeProfile.TryGet<Fog>(out var fog))
        {
            fog = VolumeProfile.Add<Fog>(false);
        }

        if(weatherPreset == WeatherPreset.Custom) // Custom Weather Preset
        {
            if(CloudIntensity==0) // Disable clouds
            {
                clouds.enable.overrideState = true; // Enable clouds override
                clouds.enable.value = false; // Disable clouds
            }
            else // Enable clouds
            {
                clouds.enable.overrideState = true; // Enable clouds override
                clouds.enable.value = true; // Enable clouds
                clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Custom; // Set clouds preset
                clouds.densityMultiplier.value = CloudIntensity; // Set clouds density
            }
            if(FogIntensity==0) // Disable fog
            {
                fog.enabled.overrideState = true; // Enable fog override
                fog.enabled.value = false; // Disable fog
            }
            else // Enable fog
            {
                fog.enabled.overrideState = true; // Enable fog override
                fog.enabled.value = true; // Enable fog
                fog.meanFreePath.value = 50+(1-FogIntensity)*200; // Set fog density (actual density is inverse of this value)
                fog.baseHeight.value = EgoVehicle.position.y; // Set fog base height
                fog.maximumHeight.value = EgoVehicle.position.y + 50; // Set fog maximum height
            }
            if(RainIntensity==0) // Disable rain
            {
                Rain.SetActive(false); // Disable rain particle precipitation
            }
            else // Enable rain
            {
                if(OptimizedWeather) // Optimize performance
                {
                    var coll = RainParticles.collision;
                    coll.enabled = false; // Disable particle colliders
                    var sub = RainParticles.subEmitters;
                    sub.enabled = false; // Disable sub-emitters
                    SubRain.SetActive(false); // Disable sub-particles
                }
                else
                {
                    SubRain.SetActive(true); // Enable sub-particles
                    var coll = RainParticles.collision;
                    coll.enabled = true; // Enable particle colliders
                    var sub = RainParticles.subEmitters;
                    sub.enabled = true; // Enable sub-emitters
                }
                Rain.SetActive(true); // Enable rain particle precipitation
                Rain.transform.position = EgoVehicle.position + new Vector3(0, 25, 0); // Set transorm of rain particle precipitation above the ego vehicle
                var rain = RainParticles.main;
                rain.maxParticles = (int)(RainIntensity*100000); // Number of raindrops per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
                var em = RainParticles.emission;
                em.rateOverTime = RainIntensity*10000; // Rate of raindrops precipitation
                var raindrop = RainSubParticles.main;
                raindrop.maxParticles = (int)(RainIntensity*100000); // Number of raindrops per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
            }
            if(SnowIntensity==0) // Disable snow
            {
                Snow.SetActive(false); // Disable snow particle precipitation
            }
            else // Enable snow
            {
                if(OptimizedWeather) // Optimize performance
                {
                    var coll = SnowParticles.collision;
                    coll.enabled = false; // Disable particle colliders
                }
                else
                {
                    var coll = SnowParticles.collision;
                    coll.enabled = true; // Enable particle colliders
                }
                Snow.SetActive(true); // Enable snow particle precipitation
                Snow.transform.position = EgoVehicle.position + new Vector3(0, 25, 0); // Set transorm of rain particle precipitation above the ego vehicle
                var snow = SnowParticles.main;
                snow.maxParticles = (int)(SnowIntensity*100000); // Number of snowflakes per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
                var em = SnowParticles.emission;
                em.rateOverTime = SnowIntensity*10000; // Rate of raindrops precipitation
            }
        }
        else if(weatherPreset == WeatherPreset.Sunny) // Sunny Weather Preset
        {
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = false; // Disable clouds
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = false; // Disable fog
            Rain.SetActive(false); // Disable rain particle precipitation
            Snow.SetActive(false); // Disable snow particle precipitation
        }
        else if(weatherPreset == WeatherPreset.Cloudy) // Cloudy Weather Preset
        {
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = true; // Enable clouds
            clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Cloudy; // Set clouds preset
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = false; // Disable fog
            Rain.SetActive(false); // Disable rain particle precipitation
            Snow.SetActive(false); // Disable snow particle precipitation
        }
        else if(weatherPreset == WeatherPreset.LightFog) // Light Fog Preset
        {
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = true; // Enable clouds
            clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Sparse; // Set clouds preset
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = true; // Enable fog
            fog.meanFreePath.value = 150; // Set fog density (actual density is inverse of this value)
            fog.baseHeight.value = EgoVehicle.position.y; // Set fog base height
            fog.maximumHeight.value = EgoVehicle.position.y + 50; // Set fog maximum height
            Rain.SetActive(false); // Disable rain particle precipitation
            Snow.SetActive(false); // Disable snow particle precipitation
        }
        else if(weatherPreset == WeatherPreset.HeavyFog) // Heavy Fog Preset
        {
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = true; // Enable clouds
            clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Sparse; // Set clouds preset
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = true; // Enable fog
            fog.meanFreePath.value = 50; // Set fog density (actual density is inverse of this value)
            fog.baseHeight.value = EgoVehicle.position.y; // Set fog base height
            fog.maximumHeight.value = EgoVehicle.position.y + 50; // Set fog maximum height
            Rain.SetActive(false); // Disable rain particle precipitation
            Snow.SetActive(false); // Disable snow particle precipitation
        }
        else if(weatherPreset == WeatherPreset.LightRain) // Light Rain Preset
        {
            if(OptimizedWeather) // Optimize performance
            {
                var coll = RainParticles.collision;
                coll.enabled = false; // Disable particle colliders
                var sub = RainParticles.subEmitters;
                sub.enabled = false; // Disable sub-emitters
                SubRain.SetActive(false); // Disable sub-particles
            }
            else
            {
                SubRain.SetActive(true); // Enable sub-particles
                var coll = RainParticles.collision;
                coll.enabled = true; // Enable particle colliders
                var sub = RainParticles.subEmitters;
                sub.enabled = true; // Enable sub-emitters
            }
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = true; // Enable clouds
            clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Overcast; // Set clouds preset
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = true; // Enable fog
            fog.meanFreePath.value = 200; // Set fog density (actual density is inverse of this value)
            fog.baseHeight.value = EgoVehicle.position.y; // Set fog base height
            fog.maximumHeight.value = EgoVehicle.position.y + 50; // Set fog maximum height
            Rain.SetActive(true); // Enable rain particle precipitation
            Rain.transform.position = EgoVehicle.position + new Vector3(0, 25, 0); // Set transorm of rain particle precipitation above the ego vehicle
            var rain = RainParticles.main;
            rain.maxParticles = 10000; // Number of raindrops per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
            var em = RainParticles.emission;
            em.rateOverTime = 1000; // Rate of raindrops precipitation
            var raindrop = RainSubParticles.main;
            raindrop.maxParticles = 10000; // Number of raindrops per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
            Snow.SetActive(false); // Disable snow particle precipitation
        }
        else if(weatherPreset == WeatherPreset.HeavyRain) // Heavy Rain Preset
        {
            if(OptimizedWeather) // Optimize performance
            {
                var coll = RainParticles.collision;
                coll.enabled = false; // Disable particle colliders
                var sub = RainParticles.subEmitters;
                sub.enabled = false; // Disable sub-emitters
                SubRain.SetActive(false); // Disable sub-particles
            }
            else
            {
                SubRain.SetActive(true); // Enable sub-particles
                var coll = RainParticles.collision;
                coll.enabled = true; // Enable particle colliders
                var sub = RainParticles.subEmitters;
                sub.enabled = true; // Enable sub-emitters
            }
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = true; // Enable clouds
            clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Stormy; // Set clouds preset
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = true; // Enable fog
            fog.meanFreePath.value = 100; // Set fog density (actual density is inverse of this value)
            fog.baseHeight.value = EgoVehicle.position.y; // Set fog base height
            fog.maximumHeight.value = EgoVehicle.position.y + 50; // Set fog maximum height
            Rain.SetActive(true); // Enable rain particle precipitation
            Rain.transform.position = EgoVehicle.position + new Vector3(0, 25, 0); // Set transorm of rain particle precipitation above the ego vehicle
            var rain = RainParticles.main;
            rain.maxParticles = 100000; // Number of raindrops per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
            var em = RainParticles.emission;
            em.rateOverTime = 10000; // Rate of raindrops precipitation
            var raindrop = RainSubParticles.main;
            raindrop.maxParticles = 100000; // Number of raindrops per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
            Snow.SetActive(false); // Disable snow particle precipitation
        }
        else if(weatherPreset == WeatherPreset.LightSnow) // Light Snow Preset
        {
            if(OptimizedWeather) // Optimize performance
            {
                var coll = SnowParticles.collision;
                coll.enabled = false; // Disable particle colliders
            }
            else
            {
                var coll = SnowParticles.collision;
                coll.enabled = true; // Enable particle colliders
            }
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = true; // Enable clouds
            clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Overcast; // Set clouds preset
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = true; // Enable fog
            fog.meanFreePath.value = 200; // Set fog density (actual density is inverse of this value)
            fog.baseHeight.value = EgoVehicle.position.y; // Set fog base height
            fog.maximumHeight.value = EgoVehicle.position.y + 50; // Set fog maximum height
            Rain.SetActive(false); // Disable rain particle precipitation
            Snow.SetActive(true); // Enable snow particle precipitation
            Snow.transform.position = EgoVehicle.position + new Vector3(0, 25, 0); // Set transorm of rain particle precipitation above the ego vehicle
            var snow = SnowParticles.main;
            snow.maxParticles = 10000; // Number of snowflakes per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
            var em = SnowParticles.emission;
            em.rateOverTime = 1000; // Rate of snoflakes precipitation
        }
        else if(weatherPreset == WeatherPreset.HeavySnow) // Heavy Snow Preset
        {
            if(OptimizedWeather) // Optimize performance
            {
                var coll = SnowParticles.collision;
                coll.enabled = false; // Disable particle colliders
            }
            else
            {
                var coll = SnowParticles.collision;
                coll.enabled = true; // Enable particle colliders
            }
            clouds.enable.overrideState = true; // Enable clouds override
            clouds.enable.value = true; // Enable clouds
            clouds.cloudPreset.value = VolumetricClouds.CloudPresets.Stormy; // Set clouds preset
            fog.enabled.overrideState = true; // Enable fog override
            fog.enabled.value = true; // Enable fog
            fog.meanFreePath.value = 100; // Set fog density (actual density is inverse of this value)
            fog.baseHeight.value = EgoVehicle.position.y; // Set fog base height
            fog.maximumHeight.value = EgoVehicle.position.y + 50; // Set fog maximum height
            Rain.SetActive(false); // Disable rain particle precipitation
            Snow.SetActive(true); // Enable snow particle precipitation
            Snow.transform.position = EgoVehicle.position + new Vector3(0, 25, 0); // Set transorm of rain particle precipitation above the ego vehicle
            var snow = SnowParticles.main;
            snow.maxParticles = 100000; // Number of snowflakes per 10,000 m^2 area (set this area by scaling the `RainParticle` gameobject)
            var em = SnowParticles.emission;
            em.rateOverTime = 10000; // Rate of snoflakes precipitation
        }
    }
}
