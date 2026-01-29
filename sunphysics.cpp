#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

// --- helpers ---
static inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Gregorian date -> days since 1970-01-01 (UTC), Howard Hinnant algorithm
static inline std::int64_t days_from_civil(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400); // [0, 399]
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1; // [0, 365]
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // [0, 146096]
    return static_cast<std::int64_t>(era) * 146097 + static_cast<std::int64_t>(doe) - 719468;
}

// Compute time difference in hours between two UTC timestamps
static inline double hours_between_utc(
    int y1,int mo1,int d1,int h1,int mi1,double s1,
    int y0,int mo0,int d0,int h0,int mi0,double s0
) {
    const std::int64_t day1 = days_from_civil(y1, (unsigned)mo1, (unsigned)d1);
    const std::int64_t day0 = days_from_civil(y0, (unsigned)mo0, (unsigned)d0);

    const double sec1 = h1 * 3600.0 + mi1 * 60.0 + s1;
    const double sec0 = h0 * 3600.0 + mi0 * 60.0 + s0;

    const double delta_seconds = (double)(day1 - day0) * 86400.0 + (sec1 - sec0);
    return delta_seconds / 3600.0;
}

// Equivalent to MATLAB sunphysics(longitude,latitude,hr,min,sec,yr,mo,day)
static std::array<double, 3> sunphysics_cpp(
    double longitude_deg, double latitude_deg,
    int hr, int minute, double sec,
    int yr, int mo, int day
) {
    // --- reference values (same as MATLAB / your spreadsheet constants) ---
    const double alpha_deg = 23.44;   // degrees
    const double phi_s_deg = 134.25;  // degrees
    const double s_rad_per_hr = ((2.0 * M_PI) / 24.0) * (366.2425 / 365.2425); // rad/hr

    // Solstice reference: 2025-12-21 00:00:00 UTC
    const int sol_y = 2025, sol_m = 12, sol_d = 21, sol_h = 0, sol_min = 0;
    const double sol_s = 0.0;

    // MATLAB: time_difference = hours(date_of_interest - solstice)
    const double time_difference_hr = hours_between_utc(
        yr, mo, day, hr, minute, sec,
        sol_y, sol_m, sol_d, sol_h, sol_min, sol_s
    );

    const double lat = deg2rad(latitude_deg);
    const double lon = deg2rad(longitude_deg);
    const double alpha = deg2rad(alpha_deg);
    const double phi_s = deg2rad(phi_s_deg);

    const double arg = s_rad_per_hr * time_difference_hr + lon - phi_s;

    const double z1 = -std::cos(lat) * std::sin(arg);
    const double z2 =  std::cos(alpha) * std::cos(lat) * std::cos(arg)
                     + std::sin(alpha) * std::sin(lat);
    const double z3 = -std::sin(alpha) * std::cos(lat) * std::cos(arg)
                     + std::cos(alpha) * std::sin(lat);

    return {z1, z2, z3};
}

static void usage(const char* prog) {
    std::cerr
        << "Usage:\n  " << prog
        << " <longitude_deg> <latitude_deg> <hour> <minute> <second> <year> <month> <day>\n\n"
        << "Example:\n  " << prog << " -105.22 39.76 12 0 0 2025 12 22\n";
}

int main(int argc, char** argv) {
    try {
        if (argc != 9) {
            usage(argv[0]);
            return 2;
        }

        const double lon = std::stod(argv[1]);
        const double lat = std::stod(argv[2]);
        const int hr     = std::stoi(argv[3]);
        const int minute = std::stoi(argv[4]);
        const double sec = std::stod(argv[5]);
        const int yr     = std::stoi(argv[6]);
        const int mo     = std::stoi(argv[7]);
        const int day    = std::stoi(argv[8]);

        // Basic input sanity checks (optional but helpful)
        if (mo < 1 || mo > 12) throw std::runtime_error("month must be 1..12");
        if (day < 1 || day > 31) throw std::runtime_error("day must be 1..31");
        if (hr < 0 || hr > 23) throw std::runtime_error("hour must be 0..23");
        if (minute < 0 || minute > 59) throw std::runtime_error("minute must be 0..59");
        if (sec < 0.0 || sec >= 60.0) throw std::runtime_error("second must be in [0,60)");
        if (lat < -90.0 || lat > 90.0) throw std::runtime_error("latitude must be [-90,90]");
        // allow lon outside [-180,180] if user wants; otherwise uncomment:
        // if (lon < -180.0 || lon > 180.0) throw std::runtime_error("longitude must be [-180,180]");

        const auto z = sunphysics_cpp(lon, lat, hr, minute, sec, yr, mo, day);

        std::cout << std::setprecision(16);
        std::cout << "z1 " << z[0] << "\n";
        std::cout << "z2 " << z[1] << "\n";
        std::cout << "z3 " << z[2] << "\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        usage(argv[0]);
        return 1;
    }
}
