#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Token {
    Number(f32),
    Ident(String),
    StringLit(String),
    LParen,
    RParen,
    LBracket,
    RBracket,
    LBrace,
    RBrace,
    Comma,
    Semicolon,
    Equals,
    Plus,
    Minus,
    Star,
    Slash,
    Percent,
    Caret,
    Colon,
    Dot,
    Hash,
    Question,
    Exclaim,
    Less,
    LessEq,
    Greater,
    GreaterEq,
    EqualEqual,
    NotEqual,
    AmpAmp,
    PipePipe,
}

pub(crate) fn tokenize(input: &str) -> Result<Vec<Token>, String> {
    let mut tokens = Vec::new();
    let mut chars = input.chars().peekable();
    while let Some(&ch) = chars.peek() {
        match ch {
            ' ' | '\t' | '\n' | '\r' => {
                chars.next();
            }
            '/' => {
                chars.next();
                match chars.peek() {
                    Some('/') => {
                        while let Some(&c) = chars.peek() {
                            chars.next();
                            if c == '\n' {
                                break;
                            }
                        }
                    }
                    Some('*') => {
                        chars.next();
                        loop {
                            match chars.next() {
                                Some('*') if chars.peek() == Some(&'/') => {
                                    chars.next();
                                    break;
                                }
                                None => return Err("unterminated block comment".into()),
                                _ => {}
                            }
                        }
                    }
                    _ => tokens.push(Token::Slash),
                }
            }
            '"' => {
                chars.next();
                let mut s = String::new();
                loop {
                    match chars.next() {
                        Some('\\') => match chars.next() {
                            Some('n') => s.push('\n'),
                            Some('t') => s.push('\t'),
                            Some('\\') => s.push('\\'),
                            Some('"') => s.push('"'),
                            Some(c) => {
                                s.push('\\');
                                s.push(c);
                            }
                            None => return Err("unterminated string".into()),
                        },
                        Some('"') => break,
                        Some(c) => s.push(c),
                        None => return Err("unterminated string".into()),
                    }
                }
                tokens.push(Token::StringLit(s));
            }
            '+' => {
                chars.next();
                tokens.push(Token::Plus);
            }
            '-' => {
                chars.next();
                tokens.push(Token::Minus);
            }
            '*' => {
                chars.next();
                tokens.push(Token::Star);
            }
            '^' => {
                chars.next();
                tokens.push(Token::Caret);
            }
            '#' => {
                chars.next();
                tokens.push(Token::Hash);
            }
            '?' => {
                chars.next();
                tokens.push(Token::Question);
            }
            '.' => {
                chars.next();
                tokens.push(Token::Dot);
            }
            '!' => {
                chars.next();
                if chars.peek() == Some(&'=') {
                    chars.next();
                    tokens.push(Token::NotEqual);
                } else {
                    tokens.push(Token::Exclaim);
                }
            }
            '<' => {
                chars.next();
                if chars.peek() == Some(&'=') {
                    chars.next();
                    tokens.push(Token::LessEq);
                } else {
                    tokens.push(Token::Less);
                }
            }
            '>' => {
                chars.next();
                if chars.peek() == Some(&'=') {
                    chars.next();
                    tokens.push(Token::GreaterEq);
                } else {
                    tokens.push(Token::Greater);
                }
            }
            '=' => {
                chars.next();
                if chars.peek() == Some(&'=') {
                    chars.next();
                    tokens.push(Token::EqualEqual);
                } else {
                    tokens.push(Token::Equals);
                }
            }
            '&' => {
                chars.next();
                if chars.peek() == Some(&'&') {
                    chars.next();
                    tokens.push(Token::AmpAmp);
                } else {
                    return Err("unexpected character: & (did you mean &&?)".into());
                }
            }
            '|' => {
                chars.next();
                if chars.peek() == Some(&'|') {
                    chars.next();
                    tokens.push(Token::PipePipe);
                } else {
                    return Err("unexpected character: | (did you mean ||?)".into());
                }
            }
            '%' => {
                chars.next();
                tokens.push(Token::Percent);
            }
            '(' => {
                chars.next();
                tokens.push(Token::LParen);
            }
            ')' => {
                chars.next();
                tokens.push(Token::RParen);
            }
            '[' => {
                chars.next();
                tokens.push(Token::LBracket);
            }
            ']' => {
                chars.next();
                tokens.push(Token::RBracket);
            }
            '{' => {
                chars.next();
                tokens.push(Token::LBrace);
            }
            '}' => {
                chars.next();
                tokens.push(Token::RBrace);
            }
            ',' => {
                chars.next();
                tokens.push(Token::Comma);
            }
            ';' => {
                chars.next();
                tokens.push(Token::Semicolon);
            }
            ':' => {
                chars.next();
                tokens.push(Token::Colon);
            }
            c if c.is_ascii_digit() || c == '.' => {
                let mut s = String::new();
                while let Some(&d) = chars.peek() {
                    if d.is_ascii_digit() || d == '.' {
                        s.push(d);
                        chars.next();
                    } else if d == 'e' || d == 'E' {
                        s.push(d);
                        chars.next();
                        if let Some(&sign) = chars.peek() {
                            if sign == '+' || sign == '-' {
                                s.push(sign);
                                chars.next();
                            }
                        }
                    } else {
                        break;
                    }
                }
                let n: f32 = s.parse().map_err(|_| format!("invalid number: {}", s))?;
                tokens.push(Token::Number(n));
            }
            c if c.is_ascii_alphabetic() || c == '_' || c == '$' => {
                let mut s = String::new();
                while let Some(&d) = chars.peek() {
                    if d.is_ascii_alphanumeric() || d == '_' {
                        s.push(d);
                        chars.next();
                    } else {
                        break;
                    }
                }
                tokens.push(Token::Ident(s));
            }
            _ => return Err(format!("unexpected character: {}", ch)),
        }
    }
    Ok(tokens)
}
