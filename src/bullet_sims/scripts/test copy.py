def encode(word):
    numbers_list = {
        "zero": 0,
        "one": 1,
        "two": 2,
        "three": 3,
        "four": 4,
        "five": 5,
        "six": 6,
        "seven": 7,
        "eight": 8,
        "nine": 9,
        "ten": 10,
        "eleven": 11,
        "twelve": 12,
        "thirteen": 13,
        "fourteen": 14,
        "fifteen": 15,
        "sixteen": 16,
        "seventeen": 17,
        "eighteen": 18,
        "nineteen": 19,
        "twenty": 20,
        "thirty": 30,
        "forty": 40,
        "fifty": 50,
        "sixty": 60,
        "seventy": 70,
        "eighty": 80,
        "ninety": 90,
        "one hundred": 100,
        "one thousand": 1000}

    words = word.split('-')
    total = 0
    temp_total = 0

    for w in words:
        w = w.lower()
        if w in numbers_list:
            if w == "hundred":
                temp_total *= numbers_list[w]
            else:
                temp_total += numbers_list[w]
        else:
            return -304

    total += temp_total

    return total


def decode(number):
    numbers_list = {
        0: "zero",
        1: "one",
        2: "two",
        3: "three",
        4: "four",
        5: "five",
        6: "six",
        7: "seven",
        8: "eight",
        9: "nine",
        10: "ten",
        11: "eleven",
        12: "twelve",
        13: "thirteen",
        14: "fourteen",
        15: "fifteen",
        16: "sixteen",
        17: "seventeen",
        18: "eighteen",
        19: "nineteen",
        20: "twenty",
        30: "thirty",
        40: "forty",
        50: "fifty",
        60: "sixty",
        70: "seventy",
        80: "eighty",
        90: "ninety",
        100: "one hundred",
        1000: "one thousand"}

    if number in numbers_list:
        return numbers_list[number]
    elif number >= 100:
        hundreds = number // 100
        remainder = number % 100
        return numbers_list[hundreds] + " " + numbers_list[100] + " " + decode(remainder)
    else:
        tens = number // 10 * 10
        ones = number % 10
        return numbers_list[tens] + "-" + numbers_list[ones]















