from similarity.astronomy.visualization import show_earth_sun


if __name__ == "__main__":
    # show_earth_sun()
    # show_earth_sun(time_scaling=86_400)  # 1 day per second
    # show_earth_sun(time_scaling=60 * 60 * 24)  # 1 day per second
    show_earth_sun(time_scaling=60 * 60 * 24 * 30)  # 1 month per second
