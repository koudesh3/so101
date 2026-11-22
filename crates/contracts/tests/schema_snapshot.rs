//! Contract 1: logs stay readable forever. The compiled so101.v1 schemas must stay
//! wire-compatible with the frozen snapshot in `proto/so101.v1.snapshot.binpb`.
//!
//! Allowed: new messages, new fields, new enum values. Not allowed: changing a field's number,
//! name, type or label; removing a field or enum value without reserving it.
//!
//! After an allowed change, refresh the snapshot deliberately:
//! `SO101_UPDATE_PROTO_SNAPSHOT=1 cargo test -p contracts --test schema_snapshot` (or `just proto-snapshot`).

use std::path::PathBuf;

use prost::Message;
use prost_types::{DescriptorProto, EnumDescriptorProto, FileDescriptorProto, FileDescriptorSet};

const PACKAGE: &str = "so101.v1";

fn snapshot_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../proto/so101.v1.snapshot.binpb")
}

fn current() -> FileDescriptorSet {
    let all = FileDescriptorSet::decode(contracts::DESCRIPTOR_SET).expect("descriptor set decodes");
    let mut files: Vec<FileDescriptorProto> =
        all.file.into_iter().filter(|f| f.package.as_deref() == Some(PACKAGE)).collect();
    // Source info (comments, spans) isn't part of the wire contract.
    for f in &mut files {
        f.source_code_info = None;
    }
    FileDescriptorSet { file: files }
}

fn reserved_field(m: &DescriptorProto, number: i32, name: &str) -> bool {
    m.reserved_range.iter().any(|r| r.start() <= number && number < r.end())
        || m.reserved_name.iter().any(|n| n == name)
}

fn reserved_value(e: &EnumDescriptorProto, number: i32, name: &str) -> bool {
    e.reserved_range.iter().any(|r| r.start() <= number && number <= r.end())
        || e.reserved_name.iter().any(|n| n == name)
}

fn check_messages(old: &[DescriptorProto], new: &[DescriptorProto], scope: &str, problems: &mut Vec<String>) {
    for om in old {
        let path = format!("{scope}.{}", om.name());
        let Some(nm) = new.iter().find(|m| m.name() == om.name()) else {
            problems.push(format!("message {path} was removed"));
            continue;
        };
        for of in &om.field {
            match nm.field.iter().find(|f| f.number() == of.number()) {
                Some(nf) => {
                    let same = nf.name() == of.name()
                        && nf.r#type() == of.r#type()
                        && nf.type_name() == of.type_name()
                        && nf.label() == of.label()
                        && nf.proto3_optional() == of.proto3_optional();
                    if !same {
                        problems.push(format!(
                            "field {path}.{} (#{}) changed: {of:?} -> {nf:?}",
                            of.name(),
                            of.number()
                        ));
                    }
                }
                None if reserved_field(nm, of.number(), of.name()) => {}
                None => problems.push(format!(
                    "field {path}.{} (#{}) was removed without `reserved {}; reserved \"{}\";`",
                    of.name(),
                    of.number(),
                    of.number(),
                    of.name()
                )),
            }
        }
        check_messages(&om.nested_type, &nm.nested_type, &path, problems);
        check_enums(&om.enum_type, &nm.enum_type, &path, problems);
    }
}

fn check_enums(old: &[EnumDescriptorProto], new: &[EnumDescriptorProto], scope: &str, problems: &mut Vec<String>) {
    for oe in old {
        let path = format!("{scope}.{}", oe.name());
        let Some(ne) = new.iter().find(|e| e.name() == oe.name()) else {
            problems.push(format!("enum {path} was removed"));
            continue;
        };
        for ov in &oe.value {
            match ne.value.iter().find(|v| v.number() == ov.number()) {
                Some(nv) if nv.name() == ov.name() => {}
                Some(nv) => {
                    problems.push(format!("enum value {path} #{} renamed {} -> {}", ov.number(), ov.name(), nv.name()))
                }
                None if reserved_value(ne, ov.number(), ov.name()) => {}
                None => problems.push(format!(
                    "enum value {path}.{} (#{}) was removed without reserving it",
                    ov.name(),
                    ov.number()
                )),
            }
        }
    }
}

#[test]
fn so101_v1_stays_wire_compatible() {
    let now = current();
    if std::env::var_os("SO101_UPDATE_PROTO_SNAPSHOT").is_some() {
        std::fs::write(snapshot_path(), now.encode_to_vec()).expect("write snapshot");
        eprintln!("wrote {}", snapshot_path().display());
    }
    let bytes = std::fs::read(snapshot_path())
        .expect("proto/so101.v1.snapshot.binpb is missing; create it with SO101_UPDATE_PROTO_SNAPSHOT=1");
    let old = FileDescriptorSet::decode(bytes.as_slice()).expect("snapshot decodes");

    let mut problems = Vec::new();
    for of in &old.file {
        let Some(nf) = now.file.iter().find(|f| f.name() == of.name()) else {
            problems.push(format!("file {} was removed", of.name()));
            continue;
        };
        check_messages(&of.message_type, &nf.message_type, PACKAGE, &mut problems);
        check_enums(&of.enum_type, &nf.enum_type, PACKAGE, &mut problems);
    }
    assert!(
        problems.is_empty(),
        "so101.v1 changed incompatibly (breaking changes go in so101.v2):\n{}",
        problems.join("\n")
    );
}

#[test]
fn the_check_catches_a_renumbered_field() {
    let now = current();
    let mut broken = now.clone();
    let field = &mut broken.file[0].message_type.iter_mut().find(|m| m.name() == "JointState").unwrap().field[0];
    field.number = Some(99);
    let mut problems = Vec::new();
    check_messages(&now.file[0].message_type, &broken.file[0].message_type, PACKAGE, &mut problems);
    assert!(problems.iter().any(|p| p.contains("JointState.t_ns")), "{problems:?}");
}
