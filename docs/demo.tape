# Terminal demo for the README. Regenerate with:  make demo
# Records the real CLI against the files in examples/; nothing is staged.

Output docs/demo.gif

Set Shell "bash"
Set FontSize 15
Set Width 1060
Set Height 430
Set Padding 18
Set Theme "GitHub Dark"
Set TypingSpeed 35ms

Hide
Type "export PATH=/opt/gfg:$PATH PS1='$ ' && clear"
Enter
Show

Type "geofence-guard check examples/field.fence examples/survey.csv | tail -4"
Enter
Sleep 2s

Type "geofence-guard check examples/field.fence examples/shortcut.csv"
Enter
Sleep 4s

Type "echo exit $?"
Enter
Sleep 1.5s

Type "clear"
Enter
Sleep 300ms

Type "geofence-guard validate core/tests/data/typo.fence   # a misspelt key"
Enter
Sleep 3s
