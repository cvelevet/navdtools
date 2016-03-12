import AppKit

// filter text-to-speech voices (only show voices w/high-quality available)
for i in NSSpeechSynthesizer.availableVoices()
{
    var attributes  = NSSpeechSynthesizer.attributesForVoice(i)
    var identifier  = String(attributes[NSVoiceIdentifier]!)
    var name        = String(attributes[NSVoiceName]!)
    var gender      = String(attributes[NSVoiceGender]!)
    var age         = String(attributes[NSVoiceAge]!)
    var locale      = String(attributes[NSVoiceLocaleIdentifier]!)
    var hasRelative = attributes["VoiceRelativeDesirability"]
    var intRelative = Int("0")
    var valRelative = 0
    if (hasRelative != nil)
    {
        intRelative = Int(String(hasRelative!))
        if (intRelative != nil)
        {
            valRelative = intRelative!
        }
    }
    while (name.utf8.count < 10)
    {
        name += " "
    }
    for _ in (1...11) // VoiceGender
    {
        gender.removeAtIndex(gender.startIndex)
    }
    while (age.utf8.count < 3)
    {
        age = " " + age
    }
    while (locale.utf8.count < 8)
    {
        locale += " "
    }
    if (hasRelative != nil && intRelative != nil && valRelative >= 13400)
    {
        print(identifier)
        print(" -> " + name, gender, age, locale, separator: "\t")
        print(" -> " + String(valRelative))
        print("")
    }
    else
    {
        print(name + ":", String(valRelative))
        print("")
    }
}
