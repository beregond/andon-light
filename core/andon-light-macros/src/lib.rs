use proc_macro::{TokenStream, TokenTree};
use syn::DeriveInput;
use syn::{Meta, Token};

// Highly inspired by strum_macros
#[proc_macro_derive(ErrorCodesEnum, attributes(code))]
pub fn error_codes_derive(input: TokenStream) -> TokenStream {
    let ast: DeriveInput = syn::parse(input).unwrap();

    let ident = &ast.ident;

    let variants = match ast.data {
        syn::Data::Enum(enum_item) => enum_item.variants,
        _ => panic!("ErrorCodes works only on Enums"),
    };

    let len = variants.len();

    let mut to_str_arms = Vec::new();
    let mut from_str_arms = Vec::new();
    let mut description_arms = Vec::new();
    let mut level_arms = Vec::new();

    for variant in variants {
        let variant_ident = &variant.ident;
        match &variant.fields {
            syn::Fields::Unit => {}
            _ => panic!("ErrorCodes only works on unit variants"),
        }

        let variant_stringified = variant_ident.to_string();
        let variant_meta = variant
            .attrs
            .iter()
            .find(|attr| attr.path().is_ident("code"));

        let mut message: Option<String> = None;
        let mut level: Option<String> = None;
        match variant_meta {
            Some(attr) => {
                let nested = attr.parse_args_with(syn::punctuated::Punctuated::<Meta, Token![,]>::parse_terminated).unwrap();
                for meta in nested {
                    match meta {
                        Meta::NameValue(meta) => {
                            if meta.path.is_ident("message") {
                                let s: syn::LitStr = match meta.value {
                                    syn::Expr::Lit(s) => match s.lit {
                                        syn::Lit::Str(s) => s,
                                        _ => panic!("message attribute must be a string"),
                                    },
                                    _ => panic!("message attribute must be a string"),
                                };
                                message = Some(s.value());
                            } else if meta.path.is_ident("level") {
                                let s: syn::LitStr = match meta.value {
                                    syn::Expr::Lit(s) => match s.lit {
                                        syn::Lit::Str(s) => s,
                                        _ => panic!("level attribute must be a string"),
                                    },
                                    _ => panic!("level attribute must be a string"),
                                };
                                level = Some(s.value());
                            } else {
                                let ident = &meta.path.get_ident().unwrap().to_string();
                                panic!("unsupported attribute - {}", ident);
                            }
                        }
                        _ => panic!("unsupported attribute"),
                    }
                }
            }
            None => panic!(
                "All variants of ErrorCodes must have a message attribute (#[code(message = \"...\")])"
            )
        };

        let message = match message {
            Some(value) => {
                if value.is_empty() {
                    panic!("ErrorCodes message attribute must not be empty")
                }
                value
            }
            None => panic!(
                "All variants of ErrorCodes must have a message attribute (#[code(message = \"...\")])"
            ),
        };

        let level = match level {
            Some(value) => {
                if value.is_empty() {
                    panic!("ErrorCodes level attribute must not be empty")
                }
                let enum_path = match value.as_str() {
                    "idle" => "andon_light::ErrorType::DeviceIdle",
                    "warn" => "andon_light::ErrorType::DeviceWarn",
                    "error" => "andon_light::ErrorType::DeviceError",
                    "system_warn" => "andon_light::ErrorType::SystemWarn",
                    "system_error" => "andon_light::ErrorType::SystemError",
                    _ => panic!("ErrorCodes level attribute must be one of: idle, warn, error, system_warn, system_error")
                };
                enum_path.to_string()
            }
            None => panic!(
                "All variants of ErrorCodes must have a level attribute (#[code(level = \"...\")])"
            ),
        };

        to_str_arms.push(quote::quote! {
            #ident::#variant_ident => #variant_stringified,
        });
        from_str_arms.push(quote::quote! {
            stringify!(#variant_ident) => Ok(#ident::#variant_ident),
        });
        description_arms.push(quote::quote! {
            #ident::#variant_ident => #message,
        });
        level_arms.push(quote::quote! {
            #ident::#variant_ident => #level,
        });
    }

    quote::quote! {
        impl #ident {
            const SIZE: usize = #len;

            pub fn as_str(&self) -> &'static str {
                match self {
                    #(#to_str_arms)*
                }
            }

            pub fn from_str(value: &str) -> Result<Self, &'static str> {
                match value {
                    #(#from_str_arms)*
                    _ => Err("Unknown error code"),
                }
            }

            pub fn description(&self) -> &'static str {
                match self {
                    #(#description_arms)*
                }
            }

            pub fn level(&self) -> &'static str {
                match self {
                    #(#level_arms)*
                }
            }
        }
    }
    .into()
}

// Currently supporting only strings and integers
// DISCLAIMER: This is a very naive implementation and makes a lot of assumptions!
// If anything breaks - be sure you check your env vars
#[proc_macro]
pub fn generate_default_from_env(input: TokenStream) -> TokenStream {
    let tokens = input.into_iter().collect::<Vec<_>>();
    match &tokens[..] {
        [TokenTree::Ident(ident), TokenTree::Punct(punct), TokenTree::Literal(literal)] => {
            if punct.to_string() != "," {
                panic!("Wrong format, try (<<ENV_VAR_NAME>>, <<DEFAULT_VALUE>>)");
            }
            let ident = ident.to_string();
            let env_value = std::env::var(ident);

            let literal = literal.to_string();

            match literal {
                payload if payload.starts_with("\"") && payload.ends_with("\"") => {
                    let default_value = payload.trim_matches('"');
                    let value = env_value.unwrap_or_else(|_| default_value.to_string());
                    quote::quote! {
                        #value
                    }
                    .into()
                }
                payload => {
                    let value = env_value
                        .unwrap_or_else(|_| payload.to_string())
                        .parse::<usize>()
                        .unwrap();
                    quote::quote! {
                        #value
                    }
                    .into()
                }
            }
        }
        _ => {
            panic!("Wrong format, try (<<ENV_VAR_NAME>>, <<DEFAULT_VALUE>>)");
        }
    }
}
