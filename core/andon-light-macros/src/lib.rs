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

    let mut to_str_arms = Vec::new();
    let mut from_str_arms = Vec::new();
    let mut description_arms = Vec::new();
    let mut level_arms = Vec::new();

    let mut codes_amount: usize = 0;

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
                                panic!("unsupported attribute - {ident}");
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
                    "idle" => "DeviceIdle",
                    "warn" => "DeviceWarn",
                    "error" => "DeviceError",
                    "system_warn" => "SystemWarn",
                    "system_error" => "SystemError",
                    "ok" => "Ok",
                    _ => panic!("ErrorCodes level attribute must be one of: ok, idle, warn, error, system_warn, system_error")
                };
                enum_path.to_string()
            }
            None => panic!(
                "All variants of ErrorCodes must have a level attribute (#[code(level = \"...\")])"
            ),
        };

        // Counting all not 'ok' codes - codes that we will actually store in codes storage
        match level.as_str() {
            "Ok" => {}
            _ => {
                codes_amount += 1;
            }
        }

        to_str_arms.push(quote::quote! {
            #ident::#variant_ident => #variant_stringified,
        });
        from_str_arms.push(quote::quote! {
            stringify!(#variant_ident) => Ok(#ident::#variant_ident),
        });
        description_arms.push(quote::quote! {
            #ident::#variant_ident => #message,
        });
        let path: syn::Path =
            syn::parse_str(format!("andon_light_core::ErrorType::{level}").as_str())
                .expect("Failed to parse path");
        let segments = path
            .segments
            .iter()
            .map(|segment| &segment.ident)
            .collect::<Vec<_>>();
        level_arms.push(quote::quote! {
            #ident::#variant_ident => #(#segments)::*,
        });
    }

    if codes_amount < 1 {
        panic!("ErrorCodes must have at least one variant, that is not 'ok' level");
    }

    let mut min_size = next_power_of_2(codes_amount);
    // Min size is used later to create FnvIndexSet, so it must be at least 2
    if min_size < 2 {
        min_size = 2;
    }

    quote::quote! {
        impl andon_light_core::ErrorCodesBase for #ident {
            const MIN_SET_SIZE: usize = #min_size;
            const CODES_AMOUNT: usize = #codes_amount;

            fn as_str(&self) -> &'static str {
                match self {
                    #(#to_str_arms)*
                }
            }

            fn level(&self) -> andon_light_core::ErrorType {
                match self {
                    #(#level_arms)*
                }
            }

            fn description(&self) -> &'static str {
                match self {
                    #(#description_arms)*
                }
            }

            fn from_str(value: &str) -> Result<Self, &'static str> {
                match value {
                    #(#from_str_arms)*
                    _ => Err("Unknown error code"),
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
pub fn default_from_env(input: TokenStream) -> TokenStream {
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

fn next_power_of_2(n: usize) -> usize {
    let mut n = n;
    n -= 1;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n += 1;
    n
}
