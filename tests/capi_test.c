#include <float.h>
#include <math.h>
#include <stdio.h>

#include "boink/boink_c_api.h"

void printVehicleState(const BoinkVehicleState* state);
void printTrackData(const BoinkTrackData* data);
void printCenterlineSample(const BoinkCenterlineSample* sample, int num);
size_t findClosestIndex(const BoinkVec3* pos, const BoinkCenterlineSample* samples, size_t samples_count);
#define PRINT_ERROR()                     \
  {                                       \
    unsigned int size;                    \
    boink_get_last_error(NULL, &size);    \
    char* err_buf = malloc(size);         \
    boink_get_last_error(err_buf, &size); \
    printf("%s\n", err_buf);              \
    free(err_buf);                        \
  }

int main()
{
#ifdef WIN32
  const char* vehicle_filename = "E:\\RepozytoriaGIT\\HackArena3.0-Physics-Engine\\Bolid_F1.glb";
  const char* track_filename = "E:\\RepozytoriaGIT\\HackArena3.0-Physics-Engine\\lowpoly_track_test_2.glb";
#else
  const char* vehicle_filename = "Bolid_F1.glb";
  const char* track_filename = "lowpoly_track_1_test_5.glb";
#endif

  unsigned int major, minor, patch;
  boink_get_engine_version(&major, &minor, &patch);
  printf("Engine verision: %d.%d.%d\n", major, minor, patch);
  boink_get_c_api_version(&major, &minor, &patch);
  printf("C API verision: %d.%d.%d\n", major, minor, patch);

  bool debug_enable = true;
  int code;

  if ((code = boink_init(debug_enable)) != BOINK_OK)
  {
    PRINT_ERROR();
    return -1;
  }

  BoinkVehicleMeshHandle mesh_handle;
  if ((code = boink_create_vehicle_mesh(vehicle_filename, &mesh_handle)) != BOINK_OK)
  {
    PRINT_ERROR();
    boink_terminate();
    return -1;
  }

  BoinkHandle handle = boink_create_race(track_filename);
  if (handle == NULL)
  {
    PRINT_ERROR();

    boink_destroy_vehicle_mesh(mesh_handle);
    boink_terminate();
    return -1;
  }
  BoinkGhostModeSettings settings;
  settings.enter_delay_ms = 5000.f;
  settings.exit_delay_ms = 10000.f;
  settings.enter_speed_max_mps = 5.f;
  settings.exit_speed_min_mps = 20.f;
  settings.until_completed_laps = 1;
  settings.vehicle_overlap_exit_delay_ms = 1000.f;

  if ((code = boink_set_ghost_mode_settings(handle, &settings)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }

  BoinkVehicleModel model;
  model.center_of_mass.x = 0.;
  model.center_of_mass.y = -0.4;
  model.center_of_mass.z = 0.;
  model.mass = 800.;
  model.max_steer_angle = 90;
  model.mesh = mesh_handle;
  model.suspension_rest_length = 0.4;
  model.wheel_radius = 0.33;

  uint64_t id0;
  if ((code = boink_spawn_vehicle(handle, &model, &id0)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }
  /*uint64_t id1;
  if ((code = boink_spawn_vehicle(handle, &model, &id1)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }*/

  if ((code = boink_despawn_vehicle(handle, id0)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }
  if ((code = boink_spawn_vehicle(handle, &model, &id0)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }

  BoinkVec3 track_pos;
  track_pos.x = 5.;
  track_pos.y = 0.;
  track_pos.z = 0.;
  if ((code = boink_set_track_position(handle, &track_pos)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }

  BoinkVec3 vehicle_pos;
  vehicle_pos.x = 0.;
  vehicle_pos.y = 13.;
  vehicle_pos.z = 3.;
  if ((code = boink_set_vehicle_position(handle, id0, &vehicle_pos)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }
  BoinkControls controls;
  controls.brake = 0.0;
  controls.steer = 0.0;
  controls.throttle = 1.;

  if ((code = boink_set_controls(handle, id0, &controls)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }

  BoinkTrackData data;
  if ((code = boink_get_track_data(handle, &data)) != BOINK_OK)
  {
    PRINT_ERROR();
    goto clear;
  }
  printTrackData(&data);

  Real prev = boink_get_time_debug();
  bool runOnce = false;
  bool runOnce2 = false;
  while (!boink_should_close_debug())
  {
    Real now = boink_get_time_debug();
    Real dt = now - prev;
    prev = now;

    if ((code = boink_step_race(handle, dt)) != BOINK_OK)
    {
      PRINT_ERROR();
      goto clear;
    }

    BoinkWeather weather_state;
    weather_state.cloudiness = 0.5f;
    weather_state.rain_intensity = 0.4f;
    weather_state.temperature_c = 10.f;

    Real dur;
    boink_get_race_duration(handle, &dur);
    if (dur > 4.f && dur < 4.6f)
    {
      if ((code = boink_set_weather(handle, &weather_state)) != BOINK_OK)
      {
        PRINT_ERROR();
        goto clear;
      }
    }

    if (!runOnce && dur > 10.)
    {
      if ((code = boink_disable_ghost_mode(handle)) != BOINK_OK)
      {
        PRINT_ERROR();
        goto clear;
      }
      runOnce = true;
    }

    if (!runOnce2 && dur > 20.)
    {
      if ((code = boink_set_ghost_mode_settings(handle, &settings)) != BOINK_OK)
      {
        PRINT_ERROR();
        goto clear;
      }
      runOnce2 = true;
    }

    struct BoinkVehicleState state;
    if ((code = boink_read_vehicle_state(handle, id0, &state)) != BOINK_OK)
    {
      PRINT_ERROR();
      goto clear;
    }

    size_t i_closeset =
        findClosestIndex(&state.chassis_position, data.centerline_samples, data.centerline_sample_count);

    // printCenterlineSample(&data.centerline_samples[i_closeset],0);

    boink_update_debug();
  }

  code = 0;

clear:
  boink_destroy_vehicle_mesh(mesh_handle);
  boink_destroy_race(handle);
  boink_terminate();
  return code;
}

size_t findClosestIndex(const BoinkVec3* pos, const BoinkCenterlineSample* samples, size_t samples_count)
{
  float smallest_length = FLT_MAX;
  size_t closest_i = 0;
  for (size_t i = 0; i < samples_count; i++)
  {
    BoinkVec3 subs;
    subs.x = (pos->x - samples[i].position.x);
    subs.y = (pos->y - samples[i].position.y);
    subs.z = (pos->z - samples[i].position.z);

    Real length2 = subs.x * subs.x + subs.y * subs.y + subs.z * subs.z;
    Real length = sqrtf(length2);

    if (length < smallest_length)
    {
      closest_i = i;
      smallest_length = length;
    }
  }

  return closest_i;
}

#define printStateString(x) printf(#x ": %s\n", x);

#define printStateReal(x) printf(#x ": %f\n", x);

#define printStateInt(x) printf(#x ": %d\n", x);

#define printStateVec3(v) printf(#v ": {%f,%f,%f}\n", v.x, v.y, v.z);

#define printStateQuat(q) printf(#q ": {%f,%f,%f,%f}\n", q.x, q.y, q.z, q.w);

void printVehicleState(const BoinkVehicleState* state)
{
  printStateReal(state->speed);
  printStateVec3(state->chassis_position);
  printStateQuat(state->vehicle_orientation);
}

void printTrackData(const BoinkTrackData* track_data)
{
  printf("TRACK_DATA\n");
  printStateString(track_data->map_id);
  printStateInt(track_data->version);
  printStateReal(track_data->lap_length_m);
  printStateInt(track_data->centerline_sample_count);

  const BoinkCenterlineSample* samples = track_data->centerline_samples;
  size_t num_samples = track_data->centerline_sample_count;

  // for(size_t i=0;i<num_samples;i++)
  //   printCenterlineSample(&samples[i],i);
}

void printCenterlineSample(const BoinkCenterlineSample* sample, int num)
{
  printf("Sample num=%d\n", num);

  printStateReal(sample->curvature_1pm);
  printStateReal(sample->bank_rad);
  printStateReal(sample->grade_rad);
  printStateReal(sample->s_m);
}
