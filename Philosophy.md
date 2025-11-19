# Philosophy of the 5113's Robot Code
>
> Authors: Kevin Hwang, Vlad Bondar, Ben DeBear, Ryan Radtke
>
## Simply

1. **It Works**
2. It's Clean
3. It's Efficient
4. It's Concise
5. It's simple
6. GYAAAAATTTT!

In that order

## Python Style

Style should follow the standard set by [PEP 8](https://peps.python.org/pep-0008/)

Some notes:

* Any style guidelines established by previously written code in this project take precedence over any guideline in PEP 8. That said, any written code that is in violation of PEP 8 and does not follow a preexisting guideline should be amended.
* The process of reformatting code to comply with PEP 8 is greatly facilitated by the tool [Black](https://pypi.org/project/black/), which **must** be applied to all code before committing
* Comments should not be needed as code should be easy to read
* Comments that do not contribute to readability are worse than no comments at all
* Previous naming conventions suited for Java should be abandoned in favor of PEP 8
* Avoid using prefixes to group names together (eg. `S_Drivetrain` or `P_AutoBalance`): the grouping of classes into packages makes this largely unnecessary
* Annotate function arguments (eg. `arcade_drive(self, forward: float, turn: float)`)
* Directories containing demo programs should be titled in all lowercase with words separated by hyphens (eg. "tank-drive")
* Blocks should never be more than 4 deep. Consider using extraction and inversion to fix

## Magicbot Recommendations

* Initialize all wpilib objects in `createObject()` and not in components. Consider using variable injection to make this easier.
* Be sure to account for any exceptions that could result from operator input
* Never interact with operator input or networktables in components themselves: this code should only be contained within `teleopPeriodic()`
* Only interact with hardware in the `execute()` method of components
* Keep robot.py clean: this should only contain variables to be injected into components in `createObjects()` and input handling in `teleopPeriodic()`

## Github Stylistic Conventions

* Commit messages should be written in the imperative mood, like a command (eg. "add Philosophy.md" rather than "added Philosophy.md" or "Philosophy.md is added")
* Commit messages should be descriptive and succinct: do not write "fix the bug" (what bug?), "improve drivetrain code" (in what way?), or "sfkljhsdkfhj" (this was a real commit message)
* Aside from main, branch names should be prefixed by the initials of the main programmer on the branch and an underscore (eg. "kh_pathweaver")
* An exception to this is competition branches, which should be named after the location of the competition (eg. "tabernacle")
* The message of any commit that is untested or dysfunctional should reflect this at the start of the message (eg. "(not working) add Philosophy.md" or "(untested) add Philosophy.md"). As shorthand, consider using "!!!" for dysfunctional commits and "???" for untested commits.
* See workflow for more information about our github conventions

_to be completed_
