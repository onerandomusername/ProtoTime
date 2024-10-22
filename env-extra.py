# from https://stackoverflow.com/a/66748991

Import("env")  # type: ignore  # noqa: F821

# Install missed package
try:
    import dotenv  # type: ignore
except ImportError:
    env.Execute("$PYTHONEXE -m pip install python-dotenv")  # type: ignore # noqa: F821
    try:
        import dotenv  # type: ignore
    except ImportError:
        print("Failed to install python-dotenv")
        exit(1)


config = dotenv.dotenv_values(".env")


LOCAL_VARS = {
    "LOGGER_LEVEL",
    "WIFI_SSID",
    "WIFI_PASS",
}

global_flags = []
local_flags = []
for var, value in config.items():
    formatted_var = ""
    if value is not None:
        formatted_var = f'-D{var}="{value}"'
    else:
        formatted_var = f"-D{var}"
    if var in LOCAL_VARS:
        local_flags.append(formatted_var)
    else:
        global_flags.append(formatted_var)

env.Append(BUILD_FLAGS=global_flags)  # type: ignore # noqa: F821
# undef SUBNETMASK // conflicts with some other libraries
env.Append(SRC_BUILD_FLAGS=local_flags)  # type: ignore # noqa: F821
