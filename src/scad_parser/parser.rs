use std::collections::HashMap;

use crate::scad_parser::ast::Node;
use crate::scad_parser::tokenizer::Token;

#[derive(Clone)]
struct FunctionDef {
    params: Vec<(String, Option<f32>)>,
    body_start: usize,
}

#[derive(Clone)]
struct ModuleDef {
    params: Vec<(String, Option<f32>)>,
    body_start: usize, // points to `{`
}

pub(crate) struct Parser {
    tokens: Vec<Token>,
    pos: usize,
    variables: HashMap<String, f32>,
    functions: HashMap<String, FunctionDef>,
    modules: HashMap<String, ModuleDef>,
    children_stack: Vec<Vec<Node>>,
}

// ── Named colors (subset of CSS/OpenSCAD colors) ──────────────────────

fn named_color(name: &str) -> Option<[f32; 4]> {
    Some(match name {
        "red" => [1.0, 0.0, 0.0, 1.0],
        "green" => [0.0, 0.5, 0.0, 1.0],
        "blue" => [0.0, 0.0, 1.0, 1.0],
        "yellow" => [1.0, 1.0, 0.0, 1.0],
        "cyan" => [0.0, 1.0, 1.0, 1.0],
        "magenta" => [1.0, 0.0, 1.0, 1.0],
        "white" => [1.0, 1.0, 1.0, 1.0],
        "black" => [0.0, 0.0, 0.0, 1.0],
        "orange" => [1.0, 0.647, 0.0, 1.0],
        "purple" => [0.502, 0.0, 0.502, 1.0],
        "gray" | "grey" => [0.502, 0.502, 0.502, 1.0],
        "pink" => [1.0, 0.753, 0.796, 1.0],
        "brown" => [0.647, 0.165, 0.165, 1.0],
        _ => return None,
    })
}

fn is_builtin_function(name: &str) -> bool {
    matches!(
        name,
        "sin"
            | "cos"
            | "tan"
            | "asin"
            | "acos"
            | "atan"
            | "atan2"
            | "abs"
            | "sign"
            | "floor"
            | "round"
            | "ceil"
            | "sqrt"
            | "exp"
            | "ln"
            | "log"
            | "pow"
            | "min"
            | "max"
    )
}

impl Parser {
    pub(crate) fn new(tokens: Vec<Token>) -> Self {
        Self {
            tokens,
            pos: 0,
            variables: HashMap::new(),
            functions: HashMap::new(),
            modules: HashMap::new(),
            children_stack: Vec::new(),
        }
    }

    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.pos)
    }

    fn next(&mut self) -> Option<Token> {
        let tok = self.tokens.get(self.pos)?.clone();
        self.pos += 1;
        Some(tok)
    }

    fn expect(&mut self, expected: &Token) -> Result<(), String> {
        match self.next() {
            Some(ref tok) if tok == expected => Ok(()),
            Some(tok) => Err(format!("expected {:?}, got {:?}", expected, tok)),
            None => Err(format!("expected {:?}, got end of input", expected)),
        }
    }

    fn consume_semicolon(&mut self) {
        if self.peek() == Some(&Token::Semicolon) {
            self.next();
        }
    }

    // ── Expression parsing (full precedence) ──────────────────────────

    fn parse_expr(&mut self) -> Result<f32, String> {
        self.parse_ternary()
    }

    fn parse_ternary(&mut self) -> Result<f32, String> {
        let cond = self.parse_or()?;
        if self.peek() == Some(&Token::Question) {
            self.next();
            let then_val = self.parse_expr()?;
            self.expect(&Token::Colon)?;
            let else_val = self.parse_expr()?;
            Ok(if cond != 0.0 { then_val } else { else_val })
        } else {
            Ok(cond)
        }
    }

    fn parse_or(&mut self) -> Result<f32, String> {
        let mut left = self.parse_and()?;
        while self.peek() == Some(&Token::PipePipe) {
            self.next();
            let right = self.parse_and()?;
            left = if left != 0.0 || right != 0.0 {
                1.0
            } else {
                0.0
            };
        }
        Ok(left)
    }

    fn parse_and(&mut self) -> Result<f32, String> {
        let mut left = self.parse_comparison()?;
        while self.peek() == Some(&Token::AmpAmp) {
            self.next();
            let right = self.parse_comparison()?;
            left = if left != 0.0 && right != 0.0 {
                1.0
            } else {
                0.0
            };
        }
        Ok(left)
    }

    fn parse_comparison(&mut self) -> Result<f32, String> {
        let left = self.parse_additive()?;
        let result = match self.peek() {
            Some(Token::Less) => {
                self.next();
                let right = self.parse_additive()?;
                if left < right { 1.0 } else { 0.0 }
            }
            Some(Token::LessEq) => {
                self.next();
                let right = self.parse_additive()?;
                if left <= right { 1.0 } else { 0.0 }
            }
            Some(Token::Greater) => {
                self.next();
                let right = self.parse_additive()?;
                if left > right { 1.0 } else { 0.0 }
            }
            Some(Token::GreaterEq) => {
                self.next();
                let right = self.parse_additive()?;
                if left >= right { 1.0 } else { 0.0 }
            }
            Some(Token::EqualEqual) => {
                self.next();
                let right = self.parse_additive()?;
                if (left - right).abs() < 1e-6 { 1.0 } else { 0.0 }
            }
            Some(Token::NotEqual) => {
                self.next();
                let right = self.parse_additive()?;
                if (left - right).abs() >= 1e-6 { 1.0 } else { 0.0 }
            }
            _ => left,
        };
        Ok(result)
    }

    fn parse_additive(&mut self) -> Result<f32, String> {
        let mut left = self.parse_multiplicative()?;
        loop {
            match self.peek() {
                Some(Token::Plus) => {
                    self.next();
                    left += self.parse_multiplicative()?;
                }
                Some(Token::Minus) => {
                    self.next();
                    left -= self.parse_multiplicative()?;
                }
                _ => break,
            }
        }
        Ok(left)
    }

    fn parse_multiplicative(&mut self) -> Result<f32, String> {
        let mut left = self.parse_unary()?;
        loop {
            match self.peek() {
                Some(Token::Star) => {
                    self.next();
                    left *= self.parse_unary()?;
                }
                Some(Token::Slash) => {
                    self.next();
                    left /= self.parse_unary()?;
                }
                Some(Token::Percent) => {
                    self.next();
                    left %= self.parse_unary()?;
                }
                Some(Token::Caret) => {
                    self.next();
                    let exp = self.parse_unary()?;
                    left = left.powf(exp);
                }
                _ => break,
            }
        }
        Ok(left)
    }

    fn parse_unary(&mut self) -> Result<f32, String> {
        match self.peek() {
            Some(Token::Minus) => {
                self.next();
                Ok(-self.parse_unary()?)
            }
            Some(Token::Exclaim) => {
                self.next();
                let val = self.parse_unary()?;
                Ok(if val == 0.0 { 1.0 } else { 0.0 })
            }
            _ => self.parse_atom(),
        }
    }

    fn parse_atom(&mut self) -> Result<f32, String> {
        match self.next() {
            Some(Token::Number(n)) => Ok(n),
            Some(Token::Ident(s)) => self.resolve_ident(&s),
            Some(Token::LParen) => {
                let val = self.parse_expr()?;
                self.expect(&Token::RParen)?;
                Ok(val)
            }
            Some(tok) => Err(format!("expected number, got {:?}", tok)),
            None => Err("expected number, got end of input".into()),
        }
    }

    fn resolve_ident(&mut self, name: &str) -> Result<f32, String> {
        match name {
            "true" => return Ok(1.0),
            "false" => return Ok(0.0),
            "PI" => return Ok(std::f32::consts::PI),
            _ => {}
        }

        // Function call?
        if self.peek() == Some(&Token::LParen) {
            if is_builtin_function(name) {
                return self.call_builtin_function(name);
            }
            if self.functions.contains_key(name) {
                return self.call_user_function(name);
            }
        }

        self.variables
            .get(name)
            .copied()
            .ok_or_else(|| format!("undefined variable: {}", name))
    }

    // ── Built-in math functions ───────────────────────────────────────

    fn call_builtin_function(&mut self, name: &str) -> Result<f32, String> {
        self.expect(&Token::LParen)?;

        let result = match name {
            "sin" => {
                let v = self.parse_expr()?;
                v.to_radians().sin()
            }
            "cos" => {
                let v = self.parse_expr()?;
                v.to_radians().cos()
            }
            "tan" => {
                let v = self.parse_expr()?;
                v.to_radians().tan()
            }
            "asin" => {
                let v = self.parse_expr()?;
                v.asin().to_degrees()
            }
            "acos" => {
                let v = self.parse_expr()?;
                v.acos().to_degrees()
            }
            "atan" => {
                let v = self.parse_expr()?;
                v.atan().to_degrees()
            }
            "abs" => {
                let v = self.parse_expr()?;
                v.abs()
            }
            "sign" => {
                let v = self.parse_expr()?;
                if v > 0.0 {
                    1.0
                } else if v < 0.0 {
                    -1.0
                } else {
                    0.0
                }
            }
            "floor" => {
                let v = self.parse_expr()?;
                v.floor()
            }
            "round" => {
                let v = self.parse_expr()?;
                v.round()
            }
            "ceil" => {
                let v = self.parse_expr()?;
                v.ceil()
            }
            "sqrt" => {
                let v = self.parse_expr()?;
                v.sqrt()
            }
            "exp" => {
                let v = self.parse_expr()?;
                v.exp()
            }
            "ln" => {
                let v = self.parse_expr()?;
                v.ln()
            }
            "log" => {
                let v = self.parse_expr()?;
                v.log10()
            }
            "pow" => {
                let base = self.parse_expr()?;
                self.expect(&Token::Comma)?;
                let exp = self.parse_expr()?;
                base.powf(exp)
            }
            "atan2" => {
                let y = self.parse_expr()?;
                self.expect(&Token::Comma)?;
                let x = self.parse_expr()?;
                y.atan2(x).to_degrees()
            }
            "min" => {
                let mut val = self.parse_expr()?;
                while self.peek() == Some(&Token::Comma) {
                    self.next();
                    val = val.min(self.parse_expr()?);
                }
                val
            }
            "max" => {
                let mut val = self.parse_expr()?;
                while self.peek() == Some(&Token::Comma) {
                    self.next();
                    val = val.max(self.parse_expr()?);
                }
                val
            }
            _ => return Err(format!("unknown function: {}", name)),
        };

        self.expect(&Token::RParen)?;
        Ok(result)
    }

    // ── User-defined function/module calls ─────────────────────────────

    fn call_user_function(&mut self, name: &str) -> Result<f32, String> {
        let func = self.functions.get(name).cloned()
            .ok_or_else(|| format!("undefined function: {}", name))?;

        let (positional, named) = self.parse_call_args()?;

        let saved = self.bind_params(&func.params, &positional, &named)?;

        let saved_pos = self.pos;
        self.pos = func.body_start;
        let result = self.parse_expr()?;
        self.pos = saved_pos;

        self.restore_vars(saved);
        Ok(result)
    }

    fn call_module(&mut self, name: &str) -> Result<Node, String> {
        let module = self.modules.get(name).cloned()
            .ok_or_else(|| format!("undefined module: {}", name))?;

        let (positional, named) = self.parse_call_args()?;

        // Parse children passed to this module call
        let children = if self.peek() == Some(&Token::Semicolon) {
            self.next();
            Vec::new()
        } else if self.peek() == Some(&Token::LBrace) {
            self.parse_block_body()?
        } else {
            vec![self.parse_statement()?]
        };

        let saved = self.bind_params(&module.params, &positional, &named)?;
        self.children_stack.push(children);

        let saved_pos = self.pos;
        self.pos = module.body_start;
        let result_nodes = self.parse_block_body()?;
        self.pos = saved_pos;

        self.children_stack.pop();
        self.restore_vars(saved);

        Ok(match result_nodes.len() {
            0 => Node::Union(Vec::new()),
            1 => result_nodes.into_iter().next().unwrap(),
            _ => Node::Union(result_nodes),
        })
    }

    fn parse_call_args(&mut self) -> Result<(Vec<f32>, HashMap<String, f32>), String> {
        self.expect(&Token::LParen)?;
        let mut positional = Vec::new();
        let mut named = HashMap::new();

        while self.peek() != Some(&Token::RParen) {
            if !positional.is_empty() || !named.is_empty() {
                if self.peek() == Some(&Token::Comma) {
                    self.next();
                }
            }
            if self.peek() == Some(&Token::RParen) {
                break;
            }

            // Check for named argument
            if let Some(Token::Ident(_)) = self.peek() {
                if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                    let arg_name = match self.next() {
                        Some(Token::Ident(s)) => s,
                        _ => unreachable!(),
                    };
                    self.next(); // =
                    let val = self.parse_expr()?;
                    named.insert(arg_name, val);
                    continue;
                }
            }

            positional.push(self.parse_expr()?);
        }
        self.expect(&Token::RParen)?;
        Ok((positional, named))
    }

    fn bind_params(
        &mut self,
        params: &[(String, Option<f32>)],
        positional: &[f32],
        named: &HashMap<String, f32>,
    ) -> Result<Vec<(String, Option<f32>)>, String> {
        let mut saved = Vec::new();
        for (i, (param_name, default)) in params.iter().enumerate() {
            let old = self.variables.get(param_name).copied();
            saved.push((param_name.clone(), old));
            let val = named
                .get(param_name)
                .copied()
                .or_else(|| positional.get(i).copied())
                .or(*default)
                .ok_or_else(|| format!("missing argument: {}", param_name))?;
            self.variables.insert(param_name.clone(), val);
        }
        Ok(saved)
    }

    fn restore_vars(&mut self, saved: Vec<(String, Option<f32>)>) {
        for (name, old) in saved {
            if let Some(v) = old {
                self.variables.insert(name, v);
            } else {
                self.variables.remove(&name);
            }
        }
    }

    fn parse_children_call(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let result = if self.peek() == Some(&Token::RParen) {
            // children() — all children
            match self.children_stack.last() {
                Some(children) if children.len() == 1 => children[0].clone(),
                Some(children) if !children.is_empty() => Node::Union(children.clone()),
                _ => Node::Union(Vec::new()),
            }
        } else {
            // children(i)
            let idx = self.parse_expr()? as usize;
            self.children_stack
                .last()
                .and_then(|c| c.get(idx).cloned())
                .unwrap_or(Node::Union(Vec::new()))
        };
        self.expect(&Token::RParen)?;
        self.consume_semicolon();
        Ok(result)
    }

    // ── Definition parsing ────────────────────────────────────────────

    /// Pre-scan the token stream for module/function definitions so they
    /// can be called before they appear in the source (forward references).
    fn prescan_definitions(&mut self) {
        let saved_pos = self.pos;
        self.pos = 0;
        while self.pos < self.tokens.len() {
            if let Some(Token::Ident(s)) = self.peek().cloned() {
                match s.as_str() {
                    "module" => {
                        self.next();
                        let _ = self.parse_module_def();
                        continue;
                    }
                    "function" => {
                        self.next();
                        let _ = self.parse_function_def();
                        continue;
                    }
                    _ => {}
                }
            }
            self.next();
        }
        self.pos = saved_pos;
    }

    fn parse_module_def(&mut self) -> Result<(), String> {
        let name = match self.next() {
            Some(Token::Ident(s)) => s,
            Some(tok) => return Err(format!("expected module name, got {:?}", tok)),
            None => return Err("expected module name".into()),
        };
        let params = self.parse_param_list()?;
        let body_start = self.pos;
        self.skip_block()?;
        self.modules.insert(name, ModuleDef { params, body_start });
        Ok(())
    }

    fn parse_function_def(&mut self) -> Result<(), String> {
        let name = match self.next() {
            Some(Token::Ident(s)) => s,
            Some(tok) => return Err(format!("expected function name, got {:?}", tok)),
            None => return Err("expected function name".into()),
        };
        let params = self.parse_param_list()?;
        self.expect(&Token::Equals)?;
        let body_start = self.pos;
        // Skip to semicolon
        let mut depth = 0i32;
        loop {
            match self.peek() {
                Some(Token::Semicolon) if depth == 0 => break,
                Some(Token::LParen | Token::LBracket) => {
                    depth += 1;
                    self.next();
                }
                Some(Token::RParen | Token::RBracket) => {
                    depth -= 1;
                    self.next();
                }
                None => return Err("unterminated function definition".into()),
                _ => {
                    self.next();
                }
            }
        }
        self.next(); // consume `;`
        self.functions
            .insert(name, FunctionDef { params, body_start });
        Ok(())
    }

    fn parse_param_list(&mut self) -> Result<Vec<(String, Option<f32>)>, String> {
        self.expect(&Token::LParen)?;
        let mut params = Vec::new();
        while self.peek() != Some(&Token::RParen) {
            if !params.is_empty() {
                self.expect(&Token::Comma)?;
            }
            let name = match self.next() {
                Some(Token::Ident(s)) => s,
                Some(tok) => return Err(format!("expected parameter name, got {:?}", tok)),
                None => return Err("expected parameter name".into()),
            };
            let default = if self.peek() == Some(&Token::Equals) {
                self.next();
                Some(self.parse_expr()?)
            } else {
                None
            };
            params.push((name, default));
        }
        self.expect(&Token::RParen)?;
        Ok(params)
    }

    /// Skip a `module` or `function` definition during the main parse
    /// (already captured by prescan).
    fn try_skip_definition(&mut self) -> Result<bool, String> {
        if let Some(Token::Ident(s)) = self.peek().cloned() {
            match s.as_str() {
                "module" => {
                    self.next();
                    self.parse_module_def()?;
                    return Ok(true);
                }
                "function" => {
                    self.next();
                    self.parse_function_def()?;
                    return Ok(true);
                }
                _ => {}
            }
        }
        Ok(false)
    }

    // ── Vector / list parsing ─────────────────────────────────────────

    fn parse_vec3(&mut self) -> Result<[f32; 3], String> {
        self.expect(&Token::LBracket)?;
        let x = self.parse_expr()?;
        self.expect(&Token::Comma)?;
        let y = self.parse_expr()?;
        self.expect(&Token::Comma)?;
        let z = self.parse_expr()?;
        self.expect(&Token::RBracket)?;
        Ok([x, y, z])
    }

    fn parse_vec2(&mut self) -> Result<[f32; 2], String> {
        self.expect(&Token::LBracket)?;
        let x = self.parse_expr()?;
        self.expect(&Token::Comma)?;
        let y = self.parse_expr()?;
        self.expect(&Token::RBracket)?;
        Ok([x, y])
    }

    fn parse_vec2_list(&mut self) -> Result<Vec<[f32; 2]>, String> {
        self.expect(&Token::LBracket)?;
        let mut points = Vec::new();
        while self.peek() != Some(&Token::RBracket) {
            if !points.is_empty() {
                self.expect(&Token::Comma)?;
            }
            points.push(self.parse_vec2()?);
        }
        self.expect(&Token::RBracket)?;
        Ok(points)
    }

    fn parse_vec3_list(&mut self) -> Result<Vec<[f32; 3]>, String> {
        self.expect(&Token::LBracket)?;
        let mut points = Vec::new();
        while self.peek() != Some(&Token::RBracket) {
            if !points.is_empty() {
                self.expect(&Token::Comma)?;
            }
            points.push(self.parse_vec3()?);
        }
        self.expect(&Token::RBracket)?;
        Ok(points)
    }

    fn parse_index_list_list(&mut self) -> Result<Vec<Vec<usize>>, String> {
        self.expect(&Token::LBracket)?;
        let mut lists = Vec::new();
        while self.peek() != Some(&Token::RBracket) {
            if !lists.is_empty() {
                self.expect(&Token::Comma)?;
            }
            lists.push(self.parse_index_list()?);
        }
        self.expect(&Token::RBracket)?;
        Ok(lists)
    }

    fn parse_index_list(&mut self) -> Result<Vec<usize>, String> {
        self.expect(&Token::LBracket)?;
        let mut indices = Vec::new();
        while self.peek() != Some(&Token::RBracket) {
            if !indices.is_empty() {
                self.expect(&Token::Comma)?;
            }
            let v = self.parse_expr()?;
            indices.push(v as usize);
        }
        self.expect(&Token::RBracket)?;
        Ok(indices)
    }

    fn parse_named_bool(&mut self, name: &str) -> Result<bool, String> {
        match self.next() {
            Some(Token::Ident(s)) if s == name => {}
            Some(tok) => return Err(format!("expected '{}', got {:?}", name, tok)),
            None => return Err(format!("expected '{}', got end of input", name)),
        }
        self.expect(&Token::Equals)?;
        match self.next() {
            Some(Token::Ident(s)) if s == "true" => Ok(true),
            Some(Token::Ident(s)) if s == "false" => Ok(false),
            Some(tok) => Err(format!("expected true/false, got {:?}", tok)),
            None => Err("expected true/false, got end of input".into()),
        }
    }

    fn parse_radius_or_diameter(&mut self) -> Result<f32, String> {
        if let Some(Token::Ident(s)) = self.peek() {
            if (s == "r" || s == "d") && self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                let is_diameter = s == "d";
                self.next();
                self.next();
                let val = self.parse_expr()?;
                return Ok(if is_diameter { val / 2.0 } else { val });
            }
        }
        self.parse_expr()
    }

    fn skip_trailing_named_args(&mut self) -> Result<(), String> {
        while self.peek() == Some(&Token::Comma) {
            self.next();
            if self.peek() == Some(&Token::RParen) {
                break;
            }
            if let Some(Token::Ident(_)) = self.peek() {
                if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                    self.next();
                    self.next();
                    self.parse_expr()?;
                    continue;
                }
            }
            self.parse_expr()?;
        }
        Ok(())
    }

    // ── Assignment parsing ────────────────────────────────────────────

    fn try_parse_assignment(&mut self) -> Result<bool, String> {
        if let Some(Token::Ident(_)) = self.peek() {
            if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                let name = match self.next() {
                    Some(Token::Ident(s)) => s,
                    _ => unreachable!(),
                };
                self.expect(&Token::Equals)?;
                let val = self.parse_expr()?;
                self.expect(&Token::Semicolon)?;
                self.variables.insert(name, val);
                return Ok(true);
            }
        }
        Ok(false)
    }

    // ── Top-level & statement parsing ─────────────────────────────────

    pub(crate) fn parse_top_level(&mut self) -> Result<Node, String> {
        self.prescan_definitions();

        let mut items = Vec::new();
        while self.peek().is_some() {
            if self.try_skip_definition()? {
                continue;
            }
            if self.try_parse_assignment()? {
                continue;
            }
            items.push(self.parse_statement()?);
        }
        if items.is_empty() {
            return Err("empty file".into());
        }
        if items.len() == 1 {
            return Ok(items.pop().unwrap());
        }
        Ok(Node::Union(items))
    }

    fn parse_statement(&mut self) -> Result<Node, String> {
        // Handle modifier characters: * (disable), # (debug), % (transparent)
        match self.peek() {
            Some(Token::Star) => {
                self.next();
                self.skip_statement()?;
                return self.parse_statement();
            }
            Some(Token::Hash) | Some(Token::Percent) => {
                self.next();
            }
            _ => {}
        }

        // Handle `!` modifier (we just parse normally)
        if self.peek() == Some(&Token::Exclaim) {
            if let Some(Token::Ident(_)) = self.tokens.get(self.pos + 1) {
                self.next();
            }
        }

        let name = match self.next() {
            Some(Token::Ident(s)) => s,
            Some(tok) => return Err(format!("expected identifier, got {:?}", tok)),
            None => return Err("expected statement, got end of input".into()),
        };

        let result = match name.as_str() {
            // 3D primitives
            "cube" => self.parse_cube(),
            "sphere" => self.parse_sphere(),
            "cylinder" => self.parse_cylinder(),
            "polyhedron" => self.parse_polyhedron(),

            // 2D primitives
            "circle" => self.parse_circle(),
            "square" => self.parse_square(),
            "polygon" => self.parse_polygon(),

            // Transforms
            "translate" => self.parse_translate(),
            "rotate" => self.parse_rotate(),
            "scale" => self.parse_scale(),
            "mirror" => self.parse_mirror(),
            "color" => self.parse_color(),

            // Extrusions
            "linear_extrude" => self.parse_linear_extrude(),
            "rotate_extrude" => self.parse_rotate_extrude(),

            // CSG
            "union" => self.parse_csg_block(CsgOp::Union),
            "difference" => self.parse_csg_block(CsgOp::Difference),
            "intersection" => self.parse_csg_block(CsgOp::Intersection),
            "hull" => self.parse_csg_block(CsgOp::Hull),
            "minkowski" => self.parse_csg_block(CsgOp::Minkowski),

            // Control flow
            "if" => self.parse_if(),
            "let" => self.parse_let(),
            "for" => self.parse_for(),

            // Children (inside module bodies)
            "children" => self.parse_children_call(),

            // Non-standard
            "repeat" => self.parse_repeat(),

            // Skip-only (no geometry)
            "echo" => {
                self.skip_call_args()?;
                self.consume_semicolon();
                self.parse_statement()
            }
            "render" => {
                self.skip_call_args()?;
                self.parse_child()
            }

            _ => {
                // Check for user-defined module call
                if self.modules.contains_key(&name) {
                    self.call_module(&name)
                } else {
                    return Err(format!("unknown command: {}", name));
                }
            }
        };

        result.map_err(|e| format!("in {}: {}", name, e))
    }

    // ── Skip helpers ──────────────────────────────────────────────────

    fn skip_statement(&mut self) -> Result<(), String> {
        if self.peek() == Some(&Token::LBrace) {
            self.skip_block()?;
        } else {
            let mut depth = 0i32;
            loop {
                match self.peek() {
                    None => break,
                    Some(Token::Semicolon) if depth == 0 => {
                        self.next();
                        break;
                    }
                    Some(Token::LParen | Token::LBracket | Token::LBrace) => {
                        depth += 1;
                        self.next();
                    }
                    Some(Token::RParen | Token::RBracket | Token::RBrace) => {
                        depth -= 1;
                        self.next();
                        if depth < 0 {
                            self.pos -= 1;
                            break;
                        }
                    }
                    _ => {
                        self.next();
                    }
                }
            }
        }
        Ok(())
    }

    fn skip_block(&mut self) -> Result<(), String> {
        self.expect(&Token::LBrace)?;
        let mut depth = 1i32;
        while depth > 0 {
            match self.next() {
                Some(Token::LBrace) => depth += 1,
                Some(Token::RBrace) => depth -= 1,
                None => return Err("unterminated block".into()),
                _ => {}
            }
        }
        Ok(())
    }

    fn skip_call_args(&mut self) -> Result<(), String> {
        self.expect(&Token::LParen)?;
        let mut depth = 1i32;
        while depth > 0 {
            match self.next() {
                Some(Token::LParen) => depth += 1,
                Some(Token::RParen) => depth -= 1,
                None => return Err("unterminated argument list".into()),
                _ => {}
            }
        }
        Ok(())
    }

    // ── 3D Primitive parsing ──────────────────────────────────────────

    fn parse_cube(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let size = if self.peek() == Some(&Token::LBracket) {
            self.parse_vec3()?
        } else {
            let s = self.parse_expr()?;
            [s, s, s]
        };

        let center = if self.peek() == Some(&Token::Comma) {
            self.next();
            self.parse_named_bool("center")?
        } else {
            false
        };
        self.expect(&Token::RParen)?;
        self.consume_semicolon();

        Ok(Node::Cube { size, center })
    }

    fn parse_sphere(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let radius = self.parse_radius_or_diameter()?;
        self.skip_trailing_named_args()?;
        self.expect(&Token::RParen)?;
        self.consume_semicolon();
        Ok(Node::Sphere { radius })
    }

    fn parse_cylinder(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let mut h: Option<f32> = None;
        let mut r: Option<f32> = None;
        let mut d: Option<f32> = None;
        let mut r1: Option<f32> = None;
        let mut r2: Option<f32> = None;
        let mut d1: Option<f32> = None;
        let mut d2: Option<f32> = None;
        let mut center = false;
        let mut positional = 0;
        let mut arg_count = 0;

        while self.peek() != Some(&Token::RParen) {
            if arg_count > 0 {
                if self.peek() == Some(&Token::Comma) {
                    self.next();
                }
            }
            if self.peek() == Some(&Token::RParen) {
                break;
            }

            if let Some(Token::Ident(name)) = self.peek().cloned() {
                if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                    self.next();
                    self.next();
                    match name.as_str() {
                        "center" => {
                            center = match self.next() {
                                Some(Token::Ident(s)) if s == "true" => true,
                                Some(Token::Ident(s)) if s == "false" => false,
                                Some(tok) => {
                                    return Err(format!("expected true/false, got {:?}", tok))
                                }
                                None => {
                                    return Err("expected true/false, got end of input".into())
                                }
                            };
                        }
                        "h" => h = Some(self.parse_expr()?),
                        "r" => r = Some(self.parse_expr()?),
                        "d" => d = Some(self.parse_expr()?),
                        "r1" => r1 = Some(self.parse_expr()?),
                        "r2" => r2 = Some(self.parse_expr()?),
                        "d1" => d1 = Some(self.parse_expr()?),
                        "d2" => d2 = Some(self.parse_expr()?),
                        _ => {
                            self.parse_expr()?;
                        }
                    }
                    arg_count += 1;
                    continue;
                }
            }

            let val = self.parse_expr()?;
            match positional {
                0 => h = Some(val),
                1 => r = Some(val),
                2 => {
                    r1 = r;
                    r2 = Some(val);
                    r = None;
                }
                _ => return Err("too many positional args for cylinder".into()),
            }
            positional += 1;
            arg_count += 1;
        }
        self.expect(&Token::RParen)?;
        self.consume_semicolon();

        let h = h.unwrap_or(1.0);
        let r1 = r1
            .or(r)
            .or(d1.map(|v| v / 2.0))
            .or(d.map(|v| v / 2.0))
            .unwrap_or(1.0);
        let r2 = r2.or(d2.map(|v| v / 2.0)).unwrap_or(r1);

        Ok(Node::Cylinder {
            h,
            r1,
            r2,
            center,
        })
    }

    fn parse_polyhedron(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let mut points: Option<Vec<[f32; 3]>> = None;
        let mut faces: Option<Vec<Vec<usize>>> = None;
        let mut arg_count = 0;

        while self.peek() != Some(&Token::RParen) {
            if arg_count > 0 {
                if self.peek() == Some(&Token::Comma) {
                    self.next();
                }
            }
            if self.peek() == Some(&Token::RParen) {
                break;
            }

            if let Some(Token::Ident(name)) = self.peek().cloned() {
                if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                    self.next();
                    self.next();
                    match name.as_str() {
                        "points" => points = Some(self.parse_vec3_list()?),
                        "faces" | "triangles" => faces = Some(self.parse_index_list_list()?),
                        _ => {
                            self.parse_expr()?;
                        }
                    }
                    arg_count += 1;
                    continue;
                }
            }

            if points.is_none() {
                points = Some(self.parse_vec3_list()?);
            } else if faces.is_none() {
                faces = Some(self.parse_index_list_list()?);
            }
            arg_count += 1;
        }
        self.expect(&Token::RParen)?;
        self.consume_semicolon();

        Ok(Node::Polyhedron {
            points: points.ok_or("polyhedron requires points")?,
            faces: faces.ok_or("polyhedron requires faces")?,
        })
    }

    // ── 2D Primitive parsing ──────────────────────────────────────────

    fn parse_circle(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let radius = self.parse_radius_or_diameter()?;
        self.skip_trailing_named_args()?;
        self.expect(&Token::RParen)?;
        self.consume_semicolon();
        Ok(Node::Circle { radius })
    }

    fn parse_square(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let size = if self.peek() == Some(&Token::LBracket) {
            self.parse_vec2()?
        } else {
            let s = self.parse_expr()?;
            [s, s]
        };

        let center = if self.peek() == Some(&Token::Comma) {
            self.next();
            self.parse_named_bool("center")?
        } else {
            false
        };
        self.expect(&Token::RParen)?;
        self.consume_semicolon();

        Ok(Node::Square { size, center })
    }

    fn parse_polygon(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let mut points: Option<Vec<[f32; 2]>> = None;
        let mut paths: Option<Vec<Vec<usize>>> = None;
        let mut arg_count = 0;

        while self.peek() != Some(&Token::RParen) {
            if arg_count > 0 {
                if self.peek() == Some(&Token::Comma) {
                    self.next();
                }
            }
            if self.peek() == Some(&Token::RParen) {
                break;
            }

            if let Some(Token::Ident(name)) = self.peek().cloned() {
                if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                    self.next();
                    self.next();
                    match name.as_str() {
                        "points" => points = Some(self.parse_vec2_list()?),
                        "paths" => paths = Some(self.parse_index_list_list()?),
                        _ => {
                            self.parse_expr()?;
                        }
                    }
                    arg_count += 1;
                    continue;
                }
            }

            if points.is_none() {
                points = Some(self.parse_vec2_list()?);
            } else if paths.is_none() {
                paths = Some(self.parse_index_list_list()?);
            }
            arg_count += 1;
        }
        self.expect(&Token::RParen)?;
        self.consume_semicolon();

        Ok(Node::Polygon {
            points: points.ok_or("polygon requires points")?,
            paths,
        })
    }

    // ── Transform parsing ─────────────────────────────────────────────

    fn parse_translate(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let offset = self.parse_vec3()?;
        self.expect(&Token::RParen)?;
        let child = self.parse_child()?;
        Ok(Node::Translate {
            offset,
            child: Box::new(child),
        })
    }

    fn parse_scale(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let factor = if self.peek() == Some(&Token::LBracket) {
            self.parse_vec3()?
        } else {
            let s = self.parse_expr()?;
            [s, s, s]
        };
        self.expect(&Token::RParen)?;
        let child = self.parse_child()?;
        Ok(Node::Scale {
            factor,
            child: Box::new(child),
        })
    }

    fn parse_rotate(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        if self.peek() == Some(&Token::LBracket) {
            let angles = self.parse_vec3()?;
            self.expect(&Token::RParen)?;
            let child = self.parse_child()?;
            Ok(Node::RotateEuler {
                angles,
                child: Box::new(child),
            })
        } else {
            let mut angle = self.parse_expr()?;
            let mut axis = [0.0, 0.0, 1.0];

            while self.peek() == Some(&Token::Comma) {
                self.next();
                if let Some(Token::Ident(name)) = self.peek().cloned() {
                    if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                        self.next();
                        self.next();
                        match name.as_str() {
                            "v" => axis = self.parse_vec3()?,
                            "a" => angle = self.parse_expr()?,
                            _ => {
                                self.parse_expr()?;
                            }
                        }
                        continue;
                    }
                }
                axis = self.parse_vec3()?;
            }

            self.expect(&Token::RParen)?;
            let child = self.parse_child()?;
            Ok(Node::RotateAxisAngle {
                axis,
                angle,
                child: Box::new(child),
            })
        }
    }

    fn parse_mirror(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let axes = self.parse_vec3()?;
        self.expect(&Token::RParen)?;
        let child = self.parse_child()?;
        Ok(Node::Mirror {
            axes,
            child: Box::new(child),
        })
    }

    fn parse_color(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let rgba = match self.peek() {
            Some(Token::StringLit(_)) => {
                let name = match self.next() {
                    Some(Token::StringLit(s)) => s,
                    _ => unreachable!(),
                };
                let mut color =
                    named_color(&name).ok_or_else(|| format!("unknown color: {}", name))?;
                if self.peek() == Some(&Token::Comma) {
                    self.next();
                    if self.peek() != Some(&Token::RParen) {
                        color[3] = self.parse_expr()?;
                    }
                }
                color
            }
            Some(Token::LBracket) => {
                self.expect(&Token::LBracket)?;
                let r = self.parse_expr()?;
                self.expect(&Token::Comma)?;
                let g = self.parse_expr()?;
                self.expect(&Token::Comma)?;
                let b = self.parse_expr()?;
                let a = if self.peek() == Some(&Token::Comma) {
                    self.next();
                    self.parse_expr()?
                } else {
                    1.0
                };
                self.expect(&Token::RBracket)?;
                [r, g, b, a]
            }
            _ => return Err("color expects a string or [r,g,b] vector".into()),
        };

        self.expect(&Token::RParen)?;
        let child = self.parse_child()?;
        Ok(Node::Color {
            rgba,
            child: Box::new(child),
        })
    }

    // ── Extrusion parsing ─────────────────────────────────────────────

    fn parse_linear_extrude(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let mut height: Option<f32> = None;
        let mut center = false;
        let mut twist = 0.0;
        let mut slices: Option<u32> = None;
        let mut positional = 0;
        let mut arg_count = 0;

        while self.peek() != Some(&Token::RParen) {
            if arg_count > 0 {
                if self.peek() == Some(&Token::Comma) {
                    self.next();
                }
            }
            if self.peek() == Some(&Token::RParen) {
                break;
            }

            if let Some(Token::Ident(name)) = self.peek().cloned() {
                if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                    self.next();
                    self.next();
                    match name.as_str() {
                        "height" => height = Some(self.parse_expr()?),
                        "center" => {
                            center = match self.next() {
                                Some(Token::Ident(s)) if s == "true" => true,
                                Some(Token::Ident(s)) if s == "false" => false,
                                Some(tok) => {
                                    return Err(format!("expected true/false, got {:?}", tok))
                                }
                                None => {
                                    return Err("expected true/false, got end of input".into())
                                }
                            };
                        }
                        "twist" => twist = self.parse_expr()?,
                        "slices" => slices = Some(self.parse_expr()? as u32),
                        _ => {
                            self.parse_expr()?;
                        }
                    }
                    arg_count += 1;
                    continue;
                }
            }

            if positional == 0 {
                height = Some(self.parse_expr()?);
            }
            positional += 1;
            arg_count += 1;
        }
        self.expect(&Token::RParen)?;
        let child = self.parse_child()?;

        Ok(Node::LinearExtrude {
            height: height.unwrap_or(1.0),
            center,
            twist,
            slices,
            child: Box::new(child),
        })
    }

    fn parse_rotate_extrude(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let mut angle = 360.0;

        while self.peek() != Some(&Token::RParen) {
            if let Some(Token::Ident(name)) = self.peek().cloned() {
                if self.tokens.get(self.pos + 1) == Some(&Token::Equals) {
                    self.next();
                    self.next();
                    match name.as_str() {
                        "angle" => angle = self.parse_expr()?,
                        _ => {
                            self.parse_expr()?;
                        }
                    }
                    if self.peek() == Some(&Token::Comma) {
                        self.next();
                    }
                    continue;
                }
            }
            break;
        }

        self.expect(&Token::RParen)?;
        let child = self.parse_child()?;

        Ok(Node::RotateExtrude {
            angle,
            child: Box::new(child),
        })
    }

    // ── CSG / grouping parsing ────────────────────────────────────────

    fn parse_csg_block(&mut self, op: CsgOp) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        self.expect(&Token::RParen)?;
        let children = self.parse_block_body()?;
        if children.is_empty() {
            return Err(format!("{:?} block has no children", op));
        }
        Ok(match op {
            CsgOp::Union => Node::Union(children),
            CsgOp::Difference => Node::Difference(children),
            CsgOp::Intersection => Node::Intersection(children),
            CsgOp::Hull => Node::Hull(children),
            CsgOp::Minkowski => Node::Minkowski(children),
        })
    }

    fn parse_block_body(&mut self) -> Result<Vec<Node>, String> {
        self.expect(&Token::LBrace)?;
        let mut children = Vec::new();
        while self.peek() != Some(&Token::RBrace) {
            if self.try_skip_definition()? {
                continue;
            }
            if self.try_parse_assignment()? {
                continue;
            }
            children.push(self.parse_statement()?);
        }
        self.expect(&Token::RBrace)?;
        Ok(children)
    }

    fn parse_child(&mut self) -> Result<Node, String> {
        if self.peek() == Some(&Token::LBrace) {
            let children = self.parse_block_body()?;
            if children.is_empty() {
                return Err("empty block".into());
            }
            if children.len() == 1 {
                return Ok(children.into_iter().next().unwrap());
            }
            Ok(Node::Union(children))
        } else {
            self.parse_statement()
        }
    }

    // ── Control flow parsing ──────────────────────────────────────────

    fn parse_if(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let cond = self.parse_expr()?;
        self.expect(&Token::RParen)?;

        if cond != 0.0 {
            let node = self.parse_child()?;
            if self.peek() == Some(&Token::Ident("else".into())) {
                self.next();
                self.skip_statement()?;
            }
            Ok(node)
        } else {
            self.skip_statement()?;
            if self.peek() == Some(&Token::Ident("else".into())) {
                self.next();
                self.parse_child()
            } else {
                Ok(Node::Union(Vec::new()))
            }
        }
    }

    fn parse_let(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;

        let mut saved: Vec<(String, Option<f32>)> = Vec::new();

        while self.peek() != Some(&Token::RParen) {
            if !saved.is_empty() {
                self.expect(&Token::Comma)?;
            }
            let name = match self.next() {
                Some(Token::Ident(s)) => s,
                Some(tok) => return Err(format!("expected variable name, got {:?}", tok)),
                None => return Err("expected variable name".into()),
            };
            self.expect(&Token::Equals)?;
            let val = self.parse_expr()?;
            let old = self.variables.get(&name).copied();
            saved.push((name.clone(), old));
            self.variables.insert(name, val);
        }
        self.expect(&Token::RParen)?;

        let result = self.parse_child()?;
        self.restore_vars(saved);
        Ok(result)
    }

    fn parse_for(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let var = match self.next() {
            Some(Token::Ident(s)) => s,
            Some(tok) => return Err(format!("expected variable name, got {:?}", tok)),
            None => return Err("expected variable name".into()),
        };
        self.expect(&Token::Equals)?;
        self.expect(&Token::LBracket)?;

        let first = self.parse_expr()?;
        let values = match self.peek() {
            Some(Token::Colon) => {
                self.next();
                let second = self.parse_expr()?;
                let (step, end) = if self.peek() == Some(&Token::Colon) {
                    self.next();
                    let end = self.parse_expr()?;
                    (second, end)
                } else {
                    (1.0, second)
                };
                self.expect(&Token::RBracket)?;

                let mut values = Vec::new();
                if step > 0.0 {
                    let mut v = first;
                    while v <= end + 1e-6 {
                        values.push(v);
                        v += step;
                    }
                } else if step < 0.0 {
                    let mut v = first;
                    while v >= end - 1e-6 {
                        values.push(v);
                        v += step;
                    }
                }
                values
            }
            _ => {
                let mut values = vec![first];
                while self.peek() == Some(&Token::Comma) {
                    self.next();
                    values.push(self.parse_expr()?);
                }
                self.expect(&Token::RBracket)?;
                values
            }
        };

        self.expect(&Token::RParen)?;

        let body_start = self.pos;
        let old_val = self.variables.get(&var).copied();

        let mut items = Vec::new();
        for val in &values {
            self.pos = body_start;
            self.variables.insert(var.clone(), *val);
            items.push(self.parse_child()?);
        }

        if let Some(old) = old_val {
            self.variables.insert(var, old);
        } else {
            self.variables.remove(&var);
        }

        if items.is_empty() {
            return Err("empty for loop range".into());
        }

        if items.len() == 1 {
            return Ok(items.pop().unwrap());
        }
        Ok(Node::Union(items))
    }

    // ── Non-standard ──────────────────────────────────────────────────

    fn parse_repeat(&mut self) -> Result<Node, String> {
        self.expect(&Token::LParen)?;
        let spacing = self.parse_vec3()?;
        self.expect(&Token::Comma)?;
        let copies_f = self.parse_vec3()?;
        let copies = [
            copies_f[0] as u32,
            copies_f[1] as u32,
            copies_f[2] as u32,
        ];
        self.expect(&Token::RParen)?;
        let child = self.parse_child()?;
        Ok(Node::Repeat {
            spacing,
            copies,
            child: Box::new(child),
        })
    }
}

#[derive(Debug, Clone, Copy)]
enum CsgOp {
    Union,
    Difference,
    Intersection,
    Hull,
    Minkowski,
}
