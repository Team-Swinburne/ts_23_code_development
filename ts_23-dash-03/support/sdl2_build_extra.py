Import("env", "projenv")

for e in [ env, projenv ]:
    # If compiler uses `-m32`, propagate it to linker.
    # Add via script, because `-Wl,-m32` does not work.
    if "-m32" in e['CCFLAGS']:
        e.Append(LINKFLAGS = ["-m32"])

#print('=====================================')
#print(env.Dump())

exec_name = "${BUILD_DIR}/${PROGNAME}${PROGSUFFIX}"

# Override unused "upload" to execute compiled binary
from SCons.Script import AlwaysBuild
AlwaysBuild(env.Alias("upload", exec_name))

# Add custom target to explorer
env.AddTarget(
    name = "execute",
    dependencies=None,
    actions = exec_name,
    title = "Execute",
    description = "Execute",
    group="General",
)