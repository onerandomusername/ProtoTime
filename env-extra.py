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

SOURCE_VARS = {
    "LOGGER_LEVEL",
}


src_flags = []
global_flags = []
for var, value in config.items():
    formatted_var = ""
    if value is not None:
        formatted_var = f'-D{var}="{value}"'
    else:
        formatted_var = f"-D{var}"
    if var in SOURCE_VARS:
        src_flags.append(formatted_var)
    else:
        global_flags.append(formatted_var)

env.Append(BUILD_SRC_FLAGS=src_flags)  # type: ignore # noqa: F821
env.Append(BUILD_FLAGS=global_flags)  # type: ignore # noqa: F821
