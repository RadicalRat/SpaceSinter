#include <stdio.h>
#include <math.h>
#include "SpiceUsr.h"

int main(int argc, char* argv[]) {
    // Check if an argument was passed to avoid a segmentation fault
    if (argc < 2) {
        printf("Error: Please provide a UTC time string (e.g., \"2026-03-05 16:45:00 UTC\")\n");
        return 1;
    }

    // 1. Load the Meta-kernel
    furnsh_c("solar_kernels.tm");

    // 2. Define Observer (Golden, CO)
    SpiceDouble lat_deg = 39.75;
    SpiceDouble lon_deg = -105.22;
    SpiceDouble alt     = 1.7; 

    // 3. Get Earth radii from the loaded PCK
    SpiceDouble radii[3], re, rp, f, obspos[3];
    SpiceInt n;
    bodvrd_c("EARTH", "RADII", 3, &n, radii);
    re = radii[0];
    rp = radii[2];
    f  = (re - rp) / re;

    // 4. Convert Lat/Lon/Alt to Rectangular (km)
    SpiceDouble lon_rad = lon_deg * rpd_c();
    SpiceDouble lat_rad = lat_deg * rpd_c();
    georec_c(lon_rad, lat_rad, alt, re, f, obspos);

    // 5. Convert UTC to ET
    SpiceDouble et;
    str2et_c(argv[1], &et);

    // 6. Calculate Azimuth and Elevation
    // Using IAU_EARTH as the observer frame center is EARTH
    SpiceDouble azlstat[6], lt;
    azlcpo_c("ELLIPSOID", "SUN", et, "LT+S", SPICEFALSE, SPICETRUE,
             obspos, "EARTH", "IAU_EARTH",
             azlstat, &lt);

    // 7. Extract and Convert
    SpiceDouble az_deg = azlstat[1] * dpr_c();
    SpiceDouble el_deg = azlstat[2] * dpr_c();

    // 8. Wrap Azimuth to [0, 360)
    az_deg = fmod(az_deg, 360.0);
    if (az_deg < 0) az_deg += 360.0;

    printf("--- Solar Pointing (Golden, CO) ---\n");
    printf("Time     : %s\n", argv[1]);
    printf("Azimuth  : %10.4f° (0=True North, 90=East)\n", az_deg);
    printf("Elevation: %10.4f° (Positive is above horizon)\n", el_deg);

    // Calculate angular velocity (rad/s)
    SpiceDouble dAz_dt = azlstat[4];
    SpiceDouble dEl_dt = azlstat[5];
    SpiceDouble el     = azlstat[2];

    // Component of azimuth rate projected onto the sphere
    SpiceDouble angular_speed = sqrt(pow(dAz_dt * cos(el), 2) + pow(dEl_dt, 2));

    // Convert speed to degrees per second
    SpiceDouble speed_deg_sec = angular_speed * dpr_c();

    // Calculate time to drift 0.1 degrees
    SpiceDouble time_step = 0.1 / speed_deg_sec;

    printf("Angular Speed: %f deg/s\n", speed_deg_sec);
    printf("Update required every %.2f seconds for 0.1° accuracy.\n", time_step);

    // --- 1. Deriving ENU Unit Vector from Az/El ---
    // Convert Az/El back to radians for trig functions
    SpiceDouble az_rad = azlstat[1]; 
    SpiceDouble el_rad = azlstat[2];

    SpiceDouble unit_enu[3];
    unit_enu[0] = sin(az_rad) * cos(el_rad); // East
    unit_enu[1] = cos(az_rad) * cos(el_rad); // North
    unit_enu[2] = sin(el_rad);               // Up

    // --- NEW STEP 2: Build the Pointing Matrix (Robot Base to Sun) ---
    // Use the "Up" vector as the reference for Axis 2 (the Tool's Left/Right axis)
    // This keeps the tool "Upright" relative to the base while pointing at the Sun
    SpiceDouble base_up[3] = {0.0, 0.0, 1.0};

    SpiceDouble arm_mat[3][3];
    // 1. Map Tool Z (3) to Sun
    // 2. Map Tool Y (2) to be perpendicular to the base UP vector
    twovec_c(unit_enu, 3, base_up, 2, arm_mat);

    SpiceDouble q_spice[4];
    m2q_c(arm_mat, q_spice);

    // Convert to Kinova/ROS [x, y, z, w]
    double q_kinova[4] = {q_spice[1], q_spice[2], q_spice[3], q_spice[0]};

    printf("--- Kinova Pointing Command ---\n");
    printf("Target ENU: [%.4f, %.4f, %.4f]\n", unit_enu[0], unit_enu[1], unit_enu[2]);
    printf("Quat(xyzw): [%.4f, %.4f, %.4f, %.4f]\n", 
        q_kinova[0], q_kinova[1], q_kinova[2], q_kinova[3]);

    // Good practice: unload kernels or clear the pool if this were a loop
    kclear_c();

    return 0;
}