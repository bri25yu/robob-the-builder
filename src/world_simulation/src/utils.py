def get_content_between(s, start_flag, end_flag):
    """
    Returns the substring from the first instance of the input start_flag
    to the first instance of end_flag.

    Parameters
    ----------
    s: str
    start_flag: str
    end_flag: str

    """
    start_i = s.find(start_flag)
    end_i = s.find(end_flag, start_i)
    return s[start_i + len(start_flag): end_i]
