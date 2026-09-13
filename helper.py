import argparse
import math


G = 6.67430e-11  # m^3 kg^-1 s^-2


def print_formula(title, formula):
    """Print a terminal-friendly Markdown/LaTeX-style formula."""
    print(f"\n{title}")
    print(f"  {formula}\n")


def print_krpc_body_info():
    """
    Print information about the active vessel's celestial body when kRPC
    is available and a connection can be established.
    """
    try:
        import krpc
    except ImportError:
        return

    try:
        connection = krpc.connect(
            name="Orbital Mechanics Calculator",
            timeout=1,
        )

        vessel = connection.space_center.active_vessel
        body = vessel.orbit.body

        mu = body.gravitational_parameter
        equatorial_radius = body.equatorial_radius
        atmosphere_depth = body.atmosphere_depth
        mass = body.mass
        sphere_of_influence = body.sphere_of_influence

        print("\nConnected to kRPC")
        print(f"Celestial body: {body.name}")

        print("\nCelestial-body parameters:")
        print(f"  Gravitational parameter (μ): "
              f"{mu:.6e} m³/s²")
        print(f"  Equatorial radius: "
              f"{equatorial_radius:.6f} m "
              f"({equatorial_radius / 1000.0:.6f} km)")
        print(f"  Atmosphere depth: "
              f"{atmosphere_depth:.6f} m "
              f"({atmosphere_depth / 1000.0:.6f} km)")
        print(f"  Mass: {mass:.6e} kg")
        print(f"  Sphere of influence: "
              f"{sphere_of_influence:.6f} m "
              f"({sphere_of_influence / 1000.0:.6f} km)")

    except Exception as error:
        # kRPC is optional, so calculation should still work without it.
        print(f"\nkRPC unavailable: {error}")


def km_to_m(value):
    return value * 1000.0


def m_to_km(value):
    return value / 1000.0


def format_time(seconds):
    if seconds < 60:
        return f"{seconds:.3f} s"

    minutes = seconds / 60
    if minutes < 60:
        return f"{minutes:.3f} min ({seconds:.3f} s)"

    hours = minutes / 60
    if hours < 24:
        return f"{hours:.3f} h ({seconds:.3f} s)"

    days = hours / 24
    return f"{days:.3f} days ({seconds:.3f} s)"


def validate_positive(name, value):
    if value <= 0:
        raise ValueError(f"{name} must be greater than zero")


def gravitational_parameter(mass):
    """Return standard gravitational parameter mu in m^3/s^2."""
    validate_positive("mass", mass)
    return G * mass


def vis_viva_velocity(mu, radius_km, semimajor_axis_km):
    """
    Vis-viva equation:

        v = sqrt(mu * (2/r - 1/a))

    radius and semimajor axis are supplied in km.
    """
    r = km_to_m(radius_km)
    a = km_to_m(semimajor_axis_km)

    validate_positive("radius", radius_km)
    validate_positive("semimajor axis", semimajor_axis_km)

    value = mu * (2.0 / r - 1.0 / a)

    if value <= 0:
        raise ValueError(
            "The supplied radius and semimajor axis do not produce a valid "
            "elliptical-orbit velocity."
        )

    return math.sqrt(value)


def circular_velocity(mu, radius_km):
    """Velocity of a circular orbit at radius_km."""
    r = km_to_m(radius_km)
    validate_positive("radius", radius_km)
    return math.sqrt(mu / r)


def orbital_period(mu, semimajor_axis_km):
    """
    Kepler's third law:

        T = 2*pi*sqrt(a^3 / mu)
    """
    a = km_to_m(semimajor_axis_km)
    validate_positive("semimajor axis", semimajor_axis_km)
    return 2.0 * math.pi * math.sqrt(a**3 / mu)


def semimajor_axis_from_period(mu, period_seconds):
    """
    Calculate semimajor axis from orbital period.

        T = 2*pi*sqrt(a^3 / mu)

    Rearranged:

        a = (mu * (T / (2*pi))^2)^(1/3)
    """
    validate_positive("gravitational parameter", mu)
    validate_positive("period", period_seconds)

    return (
        mu * (period_seconds / (2.0 * math.pi)) ** 2
    ) ** (1.0 / 3.0) / 1000.0


def elliptical_orbit_from_apsides(periapsis_km, apoapsis_km):
    """Return semimajor axis from periapsis and apoapsis radii."""
    validate_positive("periapsis radius", periapsis_km)
    validate_positive("apoapsis radius", apoapsis_km)

    if apoapsis_km < periapsis_km:
        raise ValueError("Apoapsis must be at least as large as periapsis")

    return (periapsis_km + apoapsis_km) / 2.0


def periapsis_from_apoapsis_and_a(apoapsis_km, semimajor_axis_km):
    """
    Since:

        a = (rp + ra) / 2

    then:

        rp = 2a - ra
    """
    validate_positive("apoapsis radius", apoapsis_km)
    validate_positive("semimajor axis", semimajor_axis_km)

    periapsis_km = 2.0 * semimajor_axis_km - apoapsis_km

    if periapsis_km <= 0:
        raise ValueError("The resulting periapsis radius is not positive")

    if periapsis_km > apoapsis_km:
        raise ValueError("The supplied semimajor axis is inconsistent")

    return periapsis_km


def apoapsis_from_periapsis_and_a(periapsis_km, semimajor_axis_km):
    """
    Since:

        a = (rp + ra) / 2

    then:

        ra = 2a - rp
    """
    validate_positive("periapsis radius", periapsis_km)
    validate_positive("semimajor axis", semimajor_axis_km)

    apoapsis_km = 2.0 * semimajor_axis_km - periapsis_km

    if apoapsis_km <= 0:
        raise ValueError("The resulting apoapsis radius is not positive")

    if apoapsis_km < periapsis_km:
        raise ValueError("The supplied semimajor axis is inconsistent")

    return apoapsis_km


def lowest_satellite_orbit(
    body_radius_km,
    satellite_radius_km,
    number_of_satellites,
    clearance_km=0.0,
    visibility_margin_km=0.0,
):
    """
    Calculate the lowest circular orbit for equally spaced satellites.

    The satellites are assumed to:
      - orbit in the same plane,
      - be equally spaced,
      - have identical physical radii,
      - remain at the same orbital radius.

    The returned orbit satisfies:

      1. Satellites do not touch each other.
      2. Satellites do not intersect the planet.
      3. Neighboring satellites are visible at or above the horizon.

    Parameters
    ----------
    body_radius_km:
        Radius of the planet.

    satellite_radius_km:
        Physical radius of each satellite.

    number_of_satellites:
        Number of equally spaced satellites. Must be at least 3 for
        the intended "neighbor above the horizon" configuration.

    clearance_km:
        Additional clearance above the planet's surface.

    visibility_margin_km:
        Extra clearance beyond the exact horizon condition.
        A value of zero means that the neighbor is exactly on the horizon.
        A positive value places it visibly above the horizon.

    Returns
    -------
    dict
        Orbit radius, altitude, angular spacing, neighbor distance,
        and the limiting constraint.
    """
    if not isinstance(number_of_satellites, int):
        raise ValueError("number_of_satellites must be an integer")

    if number_of_satellites < 3:
        raise ValueError(
            "At least three satellites are required for this configuration"
        )

    if not isinstance(clearance_km, (int, float)) or not math.isfinite(clearance_km):
        raise ValueError("clearance_km must be a finite number")

    if clearance_km < 0:
        raise ValueError("clearance_km must not be negative")

    if (
        not isinstance(visibility_margin_km, (int, float))
        or not math.isfinite(visibility_margin_km)
    ):
        raise ValueError("visibility_margin_km must be a finite number")

    if visibility_margin_km < 0:
        raise ValueError("visibility_margin_km must not be negative")

    n = number_of_satellites

    # Central angle between neighboring satellites.
    central_angle_rad = 2.0 * math.pi / n

    # Interior angle of the regular n-sided polygon.
    polygon_angle_rad = math.pi - central_angle_rad

    # Physical satellite-center separation must be at least two radii.
    spacing_limited_radius_km = (
        satellite_radius_km
        / math.sin(math.pi / n)
    )

    # The satellite must clear the planet's surface.
    surface_limited_radius_km = (
        body_radius_km
        + clearance_km
        + satellite_radius_km
    )

    # At the exact horizon condition, the chord joining neighboring
    # satellites is tangent to the effective planetary surface.
    visibility_limited_radius_km = (
        body_radius_km
        + clearance_km
        + visibility_margin_km
    ) / math.cos(math.pi / n)

    limiting_radii = {
        "satellite_spacing": spacing_limited_radius_km,
        "surface_clearance": surface_limited_radius_km,
        "horizon_visibility": visibility_limited_radius_km,
    }

    limiting_constraint = max(
        limiting_radii,
        key=limiting_radii.get,
    )

    orbital_radius_km = limiting_radii[limiting_constraint]

    neighbor_distance_km = (
        2.0
        * orbital_radius_km
        * math.sin(math.pi / n)
    )

    # Distance from the planet's center to the midpoint of the neighbor chord.
    chord_clearance_radius_km = (
        orbital_radius_km * math.cos(math.pi / n)
    )

    return {
        "orbital_radius_km": orbital_radius_km,
        "altitude_km": orbital_radius_km - body_radius_km,
        "number_of_satellites": n,
        "central_angle_deg": math.degrees(central_angle_rad),
        "polygon_angle_deg": math.degrees(polygon_angle_rad),
        "neighbor_distance_km": neighbor_distance_km,
        "chord_clearance_radius_km": chord_clearance_radius_km,
        "spacing_limited_radius_km": spacing_limited_radius_km,
        "surface_limited_radius_km": surface_limited_radius_km,
        "visibility_limited_radius_km": visibility_limited_radius_km,
        "limiting_constraint": limiting_constraint,
    }


def highest_soi_orbit(body_radius_km, soi_radius_km, safety_margin_km=0.0):
    """
    Return the largest orbit whose apoapsis remains inside the SOI.

    The SOI radius is measured from the planet's center, as in KSP.
    """
    validate_positive("body radius", body_radius_km)
    validate_positive("SOI radius", soi_radius_km)

    if safety_margin_km < 0:
        raise ValueError("safety margin must not be negative")

    maximum_apoapsis_km = soi_radius_km - safety_margin_km
    altitude_km = maximum_apoapsis_km - body_radius_km

    if altitude_km <= 0:
        raise ValueError(
            "The SOI radius must be greater than the body's radius "
            "plus the safety margin"
        )

    return {
        "maximum_apoapsis_radius_km": maximum_apoapsis_km,
        "maximum_apoapsis_altitude_km": altitude_km,
    }


def make_parser():
    parser = argparse.ArgumentParser(
        description="Kerbal Space Program orbital mechanics calculator"
    )

    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
    )

    # Shared body arguments
    def add_body_arguments(command):
        command.add_argument(
            "--mass",
            type=float,
            required=True,
            help="Body mass in kg",
        )

    # Vis-viva
    p = subparsers.add_parser(
        "visviva",
        help="Calculate velocity using the vis-viva equation",
    )
    add_body_arguments(p)
    p.add_argument("--radius", type=float, required=True)
    p.add_argument("--semi-major-axis", type=float, required=True)

    # Circular velocity
    p = subparsers.add_parser(
        "circular-velocity",
        help="Calculate circular orbital velocity",
    )
    add_body_arguments(p)
    p.add_argument("--radius", type=float, required=True)

    # Orbital period
    p = subparsers.add_parser(
        "period",
        help="Calculate orbital period",
    )
    add_body_arguments(p)
    p.add_argument("--semi-major-axis", type=float, required=True)

    # Semimajor axis from orbital period
    p = subparsers.add_parser(
        "semi-major-axis",
        help="Calculate semimajor axis from orbital period",
    )
    add_body_arguments(p)
    p.add_argument(
        "--period",
        type=float,
        required=True,
        help="Orbital period in seconds",
    )

    # Ellipse from apsides
    p = subparsers.add_parser(
        "ellipse",
        help="Calculate semimajor axis from periapsis and apoapsis",
    )
    p.add_argument("--periapsis", type=float, required=True)
    p.add_argument("--apoapsis", type=float, required=True)

    # Periapsis from apoapsis and a
    p = subparsers.add_parser(
        "periapsis",
        help="Calculate periapsis from apoapsis and semimajor axis",
    )
    p.add_argument("--apoapsis", type=float, required=True)
    p.add_argument("--semi-major-axis", type=float, required=True)

    # Apoapsis from periapsis and a
    p = subparsers.add_parser(
        "apoapsis",
        help="Calculate apoapsis from periapsis and semimajor axis",
    )
    p.add_argument("--periapsis", type=float, required=True)
    p.add_argument("--semi-major-axis", type=float, required=True)

    # # Lowest satellite orbit
    # p = subparsers.add_parser(
    #     "lowest-satellite-orbit",
    #     help="Calculate the lowest orbit for equally spaced satellites",
    # )
    # p.add_argument("--body-radius", type=float, required=True)
    # p.add_argument("--satellite-radius", type=float, required=True)
    # p.add_argument("--count", type=int, required=True)
    # p.add_argument("--clearance", type=float, default=0.0)
    p = subparsers.add_parser(
        "lowest-satellite-orbit",
        help="Calculate the lowest circular orbit for equally spaced satellites",
    )
    p.add_argument(
        "--body-radius",
        type=float,
        required=True,
        help="Radius of the planet in kilometres",
    )
    p.add_argument(
        "--satellite-radius",
        type=float,
        required=True,
        help="Radius of each satellite in kilometres",
    )
    p.add_argument(
        "--count",
        type=int,
        required=True,
        help="Number of equally spaced satellites; must be at least 3",
    )
    p.add_argument(
        "--clearance",
        type=float,
        default=0.0,
        help="Additional clearance above the planet surface in kilometres",
    )
    p.add_argument(
        "--visibility-margin",
        type=float,
        default=0.0,
        help=(
            "Additional distance beyond the exact horizon condition "
            "in kilometres"
        ),
    )

    # Highest SOI orbit
    p = subparsers.add_parser(
        "highest-soi-orbit",
        help="Calculate the highest orbit limited by the SOI",
    )
    p.add_argument("--body-radius", type=float, required=True)
    p.add_argument("--soi-radius", type=float, required=True)
    p.add_argument("--safety-margin", type=float, default=0.0)

    return parser


def main():
    parser = make_parser()
    args = parser.parse_args()

    print_krpc_body_info()

    try:
        if args.command == "visviva":
            print_formula(
                "Formula: Vis-viva equation",
                "v = √[ μ × (2/r − 1/a) ]",
            )
            print_formula(
                "Gravitational parameter",
                "μ = G × M",
            )

            mu = gravitational_parameter(args.mass)
            velocity = vis_viva_velocity(
                mu,
                args.radius,
                args.semi_major_axis,
            )

            print(f"Velocity: {velocity:.6f} m/s")
            print(f"Velocity: {velocity / 1000:.6f} km/s")

        elif args.command == "circular-velocity":
            print_formula(
                "Formula: Circular orbital velocity",
                "v = √(μ / r)",
            )
            print_formula(
                "Gravitational parameter",
                "μ = G × M",
            )

            mu = gravitational_parameter(args.mass)
            velocity = circular_velocity(mu, args.radius)

            print(f"Circular velocity: {velocity:.6f} m/s")
            print(f"Circular velocity: {velocity / 1000:.6f} km/s")

        elif args.command == "period":
            print_formula(
                "Formula: Orbital period",
                "T = 2π × √(a³ / μ)",
            )
            print_formula(
                "Gravitational parameter",
                "μ = G × M",
            )

            mu = gravitational_parameter(args.mass)
            period = orbital_period(mu, args.semi_major_axis)

            print(f"Orbital period: {format_time(period)}")

        elif args.command == "semi-major-axis":
            print_formula(
                "Formula: Semimajor axis from orbital period",
                "a = [ μ × (T / 2π)² ]^(1/3)",
            )
            print_formula(
                "Gravitational parameter",
                "μ = G × M",
            )

            mu = gravitational_parameter(args.mass)

            semimajor_axis = semimajor_axis_from_period(
                mu,
                args.period,
            )

            print(f"Semimajor axis: {semimajor_axis:.6f} km")

        elif args.command == "ellipse":
            print_formula(
                "Formula: Semimajor axis from apsides",
                "a = (rₚ + rₐ) / 2",
            )

            semimajor_axis = elliptical_orbit_from_apsides(
                args.periapsis,
                args.apoapsis,
            )

            print(f"Semimajor axis: {semimajor_axis:.6f} km")

        elif args.command == "periapsis":
            print_formula(
                "Formula: Periapsis from apoapsis and semimajor axis",
                "rₚ = 2a − rₐ",
            )

            periapsis = periapsis_from_apoapsis_and_a(
                args.apoapsis,
                args.semi_major_axis,
            )

            print(f"Periapsis radius: {periapsis:.6f} km")

        elif args.command == "apoapsis":
            print_formula(
                "Formula: Apoapsis from periapsis and semimajor axis",
                "rₐ = 2a − rₚ",
            )

            apoapsis = apoapsis_from_periapsis_and_a(
                args.periapsis,
                args.semi_major_axis,
            )

            print(f"Apoapsis radius: {apoapsis:.6f} km")

        elif args.command == "lowest-satellite-orbit":
            print_formula(
                "Formula: Central angle between neighboring satellites",
                "Δ = 2π / N",
            )
            print_formula(
                "Formula: Polygon interior angle",
                "θ = π − 2π / N",
            )
            print_formula(
                "Formula: Neighboring satellite distance",
                "d = 2r × sin(π / N)",
            )
            print_formula(
                "Formula: Satellite-spacing limit",
                "r ≥ rₛ / sin(π / N)",
            )
            print_formula(
                "Formula: Surface-clearance limit",
                "r ≥ R + c + rₛ",
            )
            print_formula(
                "Formula: Horizon-visibility limit",
                "r ≥ (R + c + m) / cos(π / N)",
            )
            print_formula(
                "Formula: Selected orbital radius",
                (
                    "r_orbit = max("
                    "rₛ / sin(π / N), "
                    "R + c + rₛ, "
                    "(R + c + m) / cos(π / N)"
                    ")"
                ),
            )

            result = lowest_satellite_orbit(
                body_radius_km=args.body_radius,
                satellite_radius_km=args.satellite_radius,
                number_of_satellites=args.count,
                clearance_km=args.clearance,
                visibility_margin_km=args.visibility_margin,
            )

            print(
                f"Lowest orbital radius: "
                f"{result['orbital_radius_km']:.6f} km"
            )
            print(
                f"Lowest altitude: "
                f"{result['altitude_km']:.6f} km"
            )
            print(
                f"Central angle: "
                f"{result['central_angle_deg']:.6f}°"
            )
            print(
                f"Polygon interior angle: "
                f"{result['polygon_angle_deg']:.6f}°"
            )
            print(
                f"Neighbor distance: "
                f"{result['neighbor_distance_km']:.6f} km"
            )
            print(
                f"Chord clearance radius: "
                f"{result['chord_clearance_radius_km']:.6f} km"
            )
            print(
                f"Spacing-limited radius: "
                f"{result['spacing_limited_radius_km']:.6f} km"
            )
            print(
                f"Surface-limited radius: "
                f"{result['surface_limited_radius_km']:.6f} km"
            )
            print(
                f"Visibility-limited radius: "
                f"{result['visibility_limited_radius_km']:.6f} km"
            )
            print(
                f"Limiting constraint: "
                f"{result['limiting_constraint']}"
            )

        elif args.command == "highest-soi-orbit":
            print_formula(
                "Formula: Maximum apoapsis radius",
                "rₐ,max = r_SOI − safety_margin",
            )
            print_formula(
                "Formula: Maximum apoapsis altitude",
                "hₐ,max = rₐ,max − R",
            )

            result = highest_soi_orbit(
                body_radius_km=args.body_radius,
                soi_radius_km=args.soi_radius,
                safety_margin_km=args.safety_margin,
            )

            print(
                f"Maximum apoapsis radius: "
                f"{result['maximum_apoapsis_radius_km']:.6f} km"
            )
            print(
                f"Maximum apoapsis altitude: "
                f"{result['maximum_apoapsis_altitude_km']:.6f} km"
            )

    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()


# # For example:
# python orbit.py lowest-satellite-orbit \
#     --body-radius 600 \
#     --satellite-radius 1 \
#     --count 3 \
#     --visibility-margin 10
